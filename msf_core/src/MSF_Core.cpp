/*

Copyright (c) 2010, Stephan Weiss, ASL, ETH Zurich, Switzerland
You can contact the author at <stephan dot weiss at ieee dot org>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <msf_core/MSF_Core.h>
#include "calcQ.h"
#include <msf_core/eigen_utils.h>

namespace msf_core
{

MSF_Core::MSF_Core()
{
  initialized_ = false;
  predictionMade_ = false;

  /// ros stuff
  ros::NodeHandle nh("MSF_Core");
  ros::NodeHandle pnh("~");

  pubState_ = nh.advertise<sensor_fusion_comm::DoubleArrayStamped> ("state_out", 1);
  pubCorrect_ = nh.advertise<sensor_fusion_comm::ExtEkf> ("correction", 1);
  pubPose_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose", 1);
  pubPoseCrtl_ = nh.advertise<sensor_fusion_comm::ExtState> ("ext_state", 1);
  msgState_.data.resize(nFullState_, 0);

  subImu_ = nh.subscribe("imu_state_input", 1 /*N_STATE_BUFFER*/, &MSF_Core::imuCallback, this);
  subState_ = nh.subscribe("hl_state_input", 1 /*N_STATE_BUFFER*/, &MSF_Core::stateCallback, this);

  msgCorrect_.state.resize(HLI_EKF_STATE_SIZE, 0);
  hl_state_buf_.state.resize(HLI_EKF_STATE_SIZE, 0);

  qvw_inittimer_ = 1;

  pnh.param("data_playback", data_playback_, false);
  reconfServer_ = new ReconfigureServer(ros::NodeHandle("~"));
  ReconfigureServer::CallbackType f = boost::bind(&MSF_Core::Config, this, _1, _2);
  reconfServer_->setCallback(f);
  //register dyn config list
  registerCallback(&MSF_Core::DynConfig, this);
}

MSF_Core::~MSF_Core()
{
  delete reconfServer_;
}

void MSF_Core::initialize(const Eigen::Matrix<double, 3, 1> & p, const Eigen::Matrix<double, 3, 1> & v,
                          const Eigen::Quaternion<double> & q, const Eigen::Matrix<double, 3, 1> & b_w,
                          const Eigen::Matrix<double, 3, 1> & b_a, const double & L,
                          const Eigen::Quaternion<double> & q_wv, const Eigen::Matrix<double, N_STATE, N_STATE> & P,
                          const Eigen::Matrix<double, 3, 1> & w_m, const Eigen::Matrix<double, 3, 1> & a_m,
                          const Eigen::Matrix<double, 3, 1> & g, const Eigen::Quaternion<double> & q_ci,
                          const Eigen::Matrix<double, 3, 1> & p_ci)
{
  initialized_ = false;
  predictionMade_ = false;
  qvw_inittimer_ = 1;

  // init state buffer
  for (int i = 0; i < N_STATE_BUFFER; i++)
  {
    StateBuffer_[i].reset();
  }

  idx_state_ = 0;
  idx_P_ = 0;
  idx_time_ = 0;

  State & state = StateBuffer_[idx_state_];
  state.p_ = p;
  state.v_ = v;
  state.q_ = q;
  state.b_w_ = b_w;
  state.b_a_ = b_a;
  state.L_ = L;
  state.q_wv_ = q_wv;
  state.q_ci_ = q_ci;
  state.p_ci_ = p_ci;
  state.w_m_ = w_m;
  state.q_int_ = state.q_wv_;
  state.a_m_ = a_m;
  state.time_ = ros::Time::now().toSec();

  if (P.maxCoeff() == 0 && P.minCoeff() == 0)
    StateBuffer_[idx_P_].P_ <<
        0.016580786012789, 0.012199934386656, -0.001458808893504, 0.021111179657363, 0.007427567799788, 0.000037801439852, 0.001171469788518, -0.001169015812942, 0.000103349776558, -0.000003813309102, 0.000015542937454, -0.000004252270155, -0.000344432741256, -0.000188322508425, -0.000003798930056, 0.002878474013131, 0.000479648737527, 0.000160244196007, 0.000012449379372, -0.000025211583296, -0.000029240408089, -0.000001069329869, -0.001271299967766, -0.000133670678392, -0.003059838896447
      , 0.012906597122666, 0.050841902184280, -0.001973897835999, 0.017928487134657, 0.043154792703685, 0.000622902345606, 0.002031938336114, 0.000401913571459, -0.000231214341523, -0.000016591523613, 0.000011431341737, 0.000007932426867, 0.000311267088246, -0.000201092426841, 0.000004838759439, 0.008371265702599, -0.000186683528686, 0.000139783403254, 0.000070116051011, -0.000021128179249, -0.000028597234778, -0.000006006222525, -0.002966959059502, 0.000313165520973, 0.003179854597069
      , -0.001345477564898, -0.000886479514041, 0.014171550800995, -0.002720150074738, 0.005673098074032, 0.007935105430084, 0.000687618072508, 0.000684952051662, 0.000022000355078, -0.000008608300759, -0.000000799656033, 0.000001107610267, -0.000106383032603, -0.000356814673233, -0.000068763009837, -0.000051146093497, -0.000091362447823, 0.000293945574578, -0.000256092019589, 0.000042269002771, -0.000009567778418, -0.000017167287470, 0.004592386869817, -0.001581055638926, 0.000227387610329
      , 0.020963436713918, 0.016241565921214, -0.002606622877434, 0.043695944809847, 0.008282523689966, -0.001656117837207, 0.001638402584126, -0.002060006975745, -0.001362992588971, -0.000001331527123, 0.000032032914797, 0.000004134961242, 0.000341541553429, -0.000100600014193, 0.000025055557965, 0.003723777310732, -0.000161259841873, 0.000175908029926, -0.000010843973378, -0.000001022919132, -0.000020982262562, -0.000009716850289, -0.002231080476166, -0.001033766890345, -0.003628168927273
      , 0.009314922877817, 0.046059780658109, 0.003565024589881, 0.015262116382857, 0.065035219304194, -0.001635353752413, 0.002492076189539, 0.001255538625264, -0.000034886338628, -0.000029672138211, 0.000006695719137, 0.000006779584634, 0.000273857318856, 0.000241559075524, 0.000026819562998, 0.007341077421410, -0.000245364703147, -0.000214640089519, 0.000072765069578, -0.000031941424035, 0.000014164172022, -0.000014177340183, -0.000530959567309, 0.000080230949640, 0.003376885297505
      , -0.000029025742686, 0.000535037190485, 0.007958782884182, -0.001871298319530, -0.002083832757411, 0.012983170487598, 0.000132746916981, 0.000083483650298, 0.000020140288935, -0.000001280987614, 0.000000838029756, -0.000000023238638, -0.000309256650920, 0.000094250769772, -0.000143135502707, 0.000262797080980, 0.000133734202454, 0.000025809353285, 0.000051787574678, 0.000002954414724, -0.000012648552708, -0.000004097271489, 0.002381975267107, -0.001036906319084, 0.000115868771739
      , 0.001237915701080, 0.002441754382058, 0.000642141528976, 0.001714303831639, 0.003652445463202, 0.000133021899909, 0.000491964329936, 0.000029132708361, 0.000054571029310, -0.000003531797659, 0.000002108308557, -0.000000655503604, -0.000036221301269, -0.000080404390258, -0.000002011184920, 0.000409618760249, 0.000006455600111, 0.000037893047554, 0.000004332215700, -0.000003727533693, 0.000000308858737, -0.000004128771100, 0.000121407327690, -0.000116077155506, -0.000044599164311
      , -0.001129210933568, 0.000810737713225, 0.000687013243217, -0.002320565048774, 0.001923423915051, 0.000083505758388, 0.000045906211371, 0.000464144924949, -0.000074174151652, -0.000001593433385, -0.000002820148135, 0.000001999456261, 0.000068256370057, -0.000050158974131, -0.000000228078959, 0.000046796063511, -0.000043197112362, 0.000007902785285, 0.000000020609692, 0.000001805172252, 0.000002146994103, 0.000005750401157, 0.000309103513087, 0.000176510147723, 0.000423690330719
      , 0.000118011626188, -0.000151939328593, -0.000003895302246, -0.001370909458095, 0.000050912424428, 0.000014452281684, 0.000048567151385, -0.000077773340951, 0.000550829253488, -0.000001499983629, -0.000001785224358, -0.000005364537487, 0.000036601273545, 0.000003384325422, -0.000000535444414, -0.000032994187143, -0.000004973649389, -0.000005428744590, 0.000002850997192, -0.000006378420798, -0.000000001181394, -0.000014301726522, 0.000038455607205, 0.000110350938971, -0.000142528866262
      , -0.000005270401860, -0.000021814853820, -0.000010366987197, -0.000002004330853, -0.000038399333509, -0.000001674413901, -0.000004404646641, -0.000002139516677, -0.000001756665835, 0.000002030485308, -0.000000003944807, 0.000000005740984, 0.000000210906625, 0.000000302650227, 0.000000014520529, -0.000003266286919, -0.000000158321546, -0.000000508006293, -0.000000000135721, -0.000000498539464, 0.000000163904942, 0.000000129053161, -0.000003222034988, 0.000000064481380, -0.000001109329693
      , 0.000016356223202, 0.000012074093112, -0.000001861055809, 0.000034349032581, 0.000006058258467, 0.000000706161071, 0.000001988651054, -0.000003017460220, -0.000001874017262, -0.000000012182671, 0.000002030455681, -0.000000019800818, 0.000000488355222, 0.000001489016879, 0.000000028100385, 0.000002786101595, -0.000000046249993, 0.000000097139883, 0.000000389735880, -0.000000195417410, -0.000000460262829, 0.000000210319469, -0.000002235134510, -0.000002851445699, -0.000002883729469
      , -0.000003154072126, 0.000010432789869, 0.000002047297121, 0.000005626984656, 0.000009913025254, 0.000000398401049, -0.000000326490919, 0.000002058769308, -0.000005291111547, 0.000000001086789, 0.000000001772501, 0.000002006545689, 0.000000044716134, 0.000000414518295, -0.000000135444520, 0.000001531318739, -0.000000211673436, 0.000000405677050, -0.000000796855836, -0.000000266538355, -0.000000133632439, -0.000000338622240, -0.000000150597295, -0.000000563086699, 0.000003088758497
      , -0.000348907202366, 0.000314489658858, -0.000097981489533, 0.000332751125893, 0.000276947396796, -0.000311267592250, -0.000035302086269, 0.000070545012901, 0.000036626247889, 0.000000400828580, 0.000000087733422, 0.000000120709451, 0.001026573886639, 0.000013867120528, 0.000031828760993, 0.000009746783802, -0.000458840039830, -0.000019468671558, -0.000043520866307, 0.000007245947338, 0.000003901799711, -0.000004201599512, -0.000047176373840, 0.000119567188660, 0.000003684858444
      , -0.000190283000907, -0.000192352300127, -0.000359131551235, -0.000107453347870, 0.000258576553615, 0.000091496162086, -0.000081280254994, -0.000048304910474, 0.000002800928601, 0.000000908905402, 0.000001125333299, 0.000000471832044, 0.000019874619416, 0.001029579153516, 0.000011053406779, 0.000021449316681, 0.000016006639334, -0.000412772225495, 0.000006993477540, 0.000002648721730, 0.000004792699830, -0.000004141354722, -0.000083992422256, 0.000015935718681, -0.000000338251253
      , -0.000004368584055, 0.000003124910665, -0.000067807653083, 0.000024474336501, 0.000022105549875, -0.000144033820704, -0.000002164571960, -0.000000083713348, -0.000000674226005, 0.000000019237635, 0.000000025526504, -0.000000057252892, 0.000032366581999, 0.000010736184803, 0.000111095066893, 0.000000615680626, -0.000015341510438, -0.000007700695237, -0.000023026256094, 0.000000638926195, 0.000000960343604, 0.000000817586113, -0.000026575050709, 0.000013993827719, -0.000002316938385
      , 0.002973222331656, 0.008292388147295, -0.000211655385599, 0.003951267473552, 0.006718811356807, 0.000277369882917, 0.000349425829596, -0.000014812000602, -0.000045952715508, -0.000002513020002, 0.000002692914948, 0.000001078825296, 0.000009897987444, 0.000020034595279, 0.000000809851157, 0.001554211174363, 0.000023959770856, -0.000037670361809, -0.000009320812655, -0.000004598853139, -0.000006284196194, -0.000000693801636, -0.000469324632849, 0.000014818785588, 0.000277219840791
      , 0.000476557664133, -0.000191539372645, -0.000089666716294, -0.000163721235917, -0.000235017605089, 0.000134712473215, 0.000007671308678, -0.000041648250772, -0.000005375975547, 0.000000156986772, 0.000000504340505, -0.000000198574002, -0.000458130878121, 0.000014584188938, -0.000015616513739, 0.000023678958593, 0.000535136781135, -0.000016449781236, 0.000040831795426, -0.000013702650244, -0.000000627377616, -0.000004196881223, 0.000002230529685, -0.000050724631819, -0.000004714535751
      , 0.000162219848991, 0.000116427796874, 0.000292562152669, 0.000173404902614, -0.000249216364740, 0.000026816594889, 0.000036367682776, 0.000005763510102, -0.000005320926337, -0.000000071291000, -0.000000112152457, 0.000000334342568, -0.000022684595881, -0.000410859858969, -0.000007890929454, -0.000040454023111, -0.000011131820455, 0.000458907544194, -0.000005285694195, 0.000002246982110, -0.000002222041169, 0.000001951461640, 0.000047488638766, -0.000029510929794, 0.000005816436594
      , 0.000010794825884, 0.000058045653749, -0.000260506684499, -0.000007544850373, 0.000048451414581, 0.000048500128303, 0.000002555777025, -0.000001118968589, 0.000001725856751, 0.000000113523451, 0.000000356160739, -0.000000287211392, -0.000041197824317, 0.000004749859562, -0.000021745597805, -0.000011794173035, 0.000040317421040, -0.000001104681255, 0.000325476240984, 0.000006084247746, -0.000006253095726, -0.000005627495374, 0.000013663440542, -0.000012536337446, 0.000000477230568
      , -0.000028222744852, -0.000029726624789, 0.000042365440829, -0.000004529013669, -0.000041974513687, 0.000002547416367, -0.000004149622895, 0.000001656132079, -0.000006464228083, -0.000000593440587, -0.000000063566120, -0.000000230872869, 0.000007212338790, 0.000002222629345, 0.000000642817161, -0.000006111733946, -0.000013813495990, 0.000002643879751, 0.000005887006479, 0.000020142991502, -0.000000692093175, -0.000000188761575, 0.000017519903352, -0.000002456326732, 0.000001576856355
      , -0.000026132063406, -0.000024675067133, -0.000008452766004, -0.000014350608058, 0.000014404004024, -0.000011620075371, 0.000000539065468, 0.000001829895964, -0.000000462890431, 0.000000223093202, -0.000000499925964, -0.000000094710754, 0.000003954308159, 0.000004249241909, 0.000000876422290, -0.000005419924437, -0.000001021458192, -0.000002052781175, -0.000007397128908, -0.000000347703730, 0.000021540076832, 0.000001455562847, 0.000005351749933, 0.000020079632692, 0.000006997090317
      , 0.000001606076924, 0.000001031428045, -0.000015843471685, -0.000005357648114, -0.000007152430254, -0.000003359339850, -0.000003466742259, 0.000005980188844, -0.000014512044407, 0.000000136766387, 0.000000188396487, -0.000000299050190, -0.000004280062694, -0.000005018186182, 0.000000751147421, 0.000000382366121, -0.000004319412270, 0.000002858658354, -0.000005774838189, -0.000000199234914, 0.000001477444848, 0.000021955531390, -0.000005912741153, 0.000006848954650, 0.000000718992109
      , -0.001250410021685, -0.002465752118803, 0.004640769479530, -0.002397333962665, 0.000543954908379, 0.002370095810071, 0.000159513911164, 0.000327435894035, 0.000051354259180, -0.000002658607585, -0.000001766738193, -0.000000182288648, -0.000049404478395, -0.000084546262756, -0.000026628375388, -0.000398670523051, 0.000000139079122, 0.000048715190023, 0.000014902392001, 0.000017378375266, 0.000005675773452, -0.000005943594846, 0.013030218966816, 0.002362333360404, 0.000426396397327
      , -0.000130856879780, 0.000387010914370, -0.001570485481930, -0.001207751008090, 0.000021063199750, -0.001030927710933, -0.000109925957135, 0.000181001368406, 0.000107869867108, 0.000000177851848, -0.000002935702240, -0.000000493441232, 0.000119019560571, 0.000014103264454, 0.000013824858652, 0.000027253599949, -0.000051452899775, -0.000028435304764, -0.000013422029969, -0.000002043413021, 0.000020290127027, 0.000006914337519, 0.002362694187196, 0.016561843614191, 0.000974154946980
      , -0.002974278550351, 0.003344054784873, 0.000125156378167, -0.003468124255435, 0.003442635413150, 0.000109148337164, -0.000076026755915, 0.000385370025486, -0.000148952839125, -0.000000760036995, -0.000002603545684, 0.000003064524894, 0.000001812974918, -0.000002381321630, -0.000002469614200, 0.000309057481545, -0.000004492645187, 0.000007689077401, 0.000001009062356, 0.000001877737433, 0.000007317725714, 0.000000467906597, 0.000372138697091, 0.000966188804360, 0.011550623767300;
  else
    StateBuffer_[idx_P_].P_ = P;

	// constants
  g_ = g;

  // buffer for vision failure check
  qvw_inittimer_ = 1;
  qbuff_ = Eigen::Matrix<double, nBuff_, 4>::Constant(0);

  // init external propagation
  msgCorrect_.header.stamp = ros::Time::now();
  msgCorrect_.header.seq = 0;
  msgCorrect_.angular_velocity.x = 0;
  msgCorrect_.angular_velocity.y = 0;
  msgCorrect_.angular_velocity.z = 0;
  msgCorrect_.linear_acceleration.x = 0;
  msgCorrect_.linear_acceleration.y = 0;
  msgCorrect_.linear_acceleration.z = 0;
  msgCorrect_.state[0] = state.p_[0];
  msgCorrect_.state[1] = state.p_[1];
  msgCorrect_.state[2] = state.p_[2];
  msgCorrect_.state[3] = state.v_[0];
  msgCorrect_.state[4] = state.v_[1];
  msgCorrect_.state[5] = state.v_[2];
  msgCorrect_.state[6] = state.q_.w();
  msgCorrect_.state[7] = state.q_.x();
  msgCorrect_.state[8] = state.q_.y();
  msgCorrect_.state[9] = state.q_.z();
  msgCorrect_.state[10] = state.b_w_[0];
  msgCorrect_.state[11] = state.b_w_[1];
  msgCorrect_.state[12] = state.b_w_[2];
  msgCorrect_.state[13] = state.b_a_[0];
  msgCorrect_.state[14] = state.b_a_[1];
  msgCorrect_.state[15] = state.b_a_[2];
  msgCorrect_.flag = sensor_fusion_comm::ExtEkf::initialization;
  pubCorrect_.publish(msgCorrect_);

  // increase state pointers
  idx_state_++;
  idx_P_++;

  initialized_ = true;
}


void MSF_Core::imuCallback(const sensor_msgs::ImuConstPtr & msg)
{

  if (!initialized_)
    return; // // early abort // //

  StateBuffer_[idx_state_].time_ = msg->header.stamp.toSec();

  static int seq = 0;

  // get inputs
  StateBuffer_[idx_state_].a_m_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
  StateBuffer_[idx_state_].w_m_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

  // remove acc spikes (TODO: find a cleaner way to do this)
  static Eigen::Matrix<double, 3, 1> last_am = Eigen::Matrix<double, 3, 1>(0, 0, 0);
  if (StateBuffer_[idx_state_].a_m_.norm() > 50)
    StateBuffer_[idx_state_].a_m_ = last_am;
  else
    last_am = StateBuffer_[idx_state_].a_m_;

  if (!predictionMade_)
  {
    if (fabs(StateBuffer_[(unsigned char)(idx_state_)].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_) > 5)
    {
      ROS_WARN_STREAM_THROTTLE(2, "large time-gap re-initializing to last state\n");
      StateBuffer_[(unsigned char)(idx_state_ - 1)].time_ = StateBuffer_[(idx_state_)].time_;
      return; // // early abort // // (if timegap too big)
    }
  }

  propagateState(StateBuffer_[idx_state_].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_);
  predictProcessCovariance(StateBuffer_[idx_P_].time_ - StateBuffer_[(unsigned char)(idx_P_ - 1)].time_);

  checkForNumeric((double*)(&StateBuffer_[idx_state_ - 1].p_[0]), 3, "prediction p");

  predictionMade_ = true;

  msgPose_.header.stamp = msg->header.stamp;
  msgPose_.header.seq = msg->header.seq;

  StateBuffer_[(unsigned char)(idx_state_ - 1)].toPoseMsg(msgPose_);
  pubPose_.publish(msgPose_);

  msgPoseCtrl_.header = msgPose_.header;
  StateBuffer_[(unsigned char)(idx_state_ - 1)].toExtStateMsg(msgPoseCtrl_);
  pubPoseCrtl_.publish(msgPoseCtrl_);

  seq++;
}


void MSF_Core::stateCallback(const sensor_fusion_comm::ExtEkfConstPtr & msg)
{

  if (!initialized_)
    return; // // early abort // //

  StateBuffer_[idx_state_].time_ = msg->header.stamp.toSec();

  static int seq = 0;

  // get inputs
  StateBuffer_[idx_state_].a_m_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
  StateBuffer_[idx_state_].w_m_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

  // remove acc spikes (TODO: find a cleaner way to do this)
  static Eigen::Matrix<double, 3, 1> last_am = Eigen::Matrix<double, 3, 1>(0, 0, 0);
  if (StateBuffer_[idx_state_].a_m_.norm() > 50)
    StateBuffer_[idx_state_].a_m_ = last_am;
  else
    last_am = StateBuffer_[idx_state_].a_m_;

  if (!predictionMade_)
  {
    if (fabs(StateBuffer_[(idx_state_)].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_) > 5)
    {
      ROS_WARN_STREAM_THROTTLE(2, "large time-gap re-initializing to last state\n");
      StateBuffer_[(unsigned char)(idx_state_ - 1)].time_ = StateBuffer_[(idx_state_)].time_;
      StateBuffer_[(unsigned char)(idx_state_)].time_ = 0;
      return; // // early abort // // (if timegap too big)
    }
  }

  int32_t flag = msg->flag;
  if (data_playback_)
    flag = sensor_fusion_comm::ExtEkf::ignore_state;

  bool isnumeric = true;
  if (flag == sensor_fusion_comm::ExtEkf::current_state)
    isnumeric = checkForNumeric(&msg->state[0], 10, "before prediction p,v,q");

  isnumeric = checkForNumeric((double*)(&StateBuffer_[idx_state_].p_[0]), 3, "before prediction p");

  if (flag == sensor_fusion_comm::ExtEkf::current_state && isnumeric) // state propagation is made externally, so we read the actual state
  {
    StateBuffer_[idx_state_].p_ = Eigen::Matrix<double, 3, 1>(msg->state[0], msg->state[1], msg->state[2]);
    StateBuffer_[idx_state_].v_ = Eigen::Matrix<double, 3, 1>(msg->state[3], msg->state[4], msg->state[5]);
    StateBuffer_[idx_state_].q_ = Eigen::Quaternion<double>(msg->state[6], msg->state[7], msg->state[8], msg->state[9]);
    StateBuffer_[idx_state_].q_.normalize();

    // zero props:
    StateBuffer_[idx_state_].b_w_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].b_w_;
    StateBuffer_[idx_state_].b_a_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].b_a_;
    StateBuffer_[idx_state_].L_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].L_;
    StateBuffer_[idx_state_].q_wv_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].q_wv_;
    StateBuffer_[idx_state_].q_ci_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].q_ci_;
    StateBuffer_[idx_state_].p_ci_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].p_ci_;
    idx_state_++;

    hl_state_buf_ = *msg;
  }
  else if (flag == sensor_fusion_comm::ExtEkf::ignore_state || !isnumeric) // otherwise let's do the state prop. here
    propagateState(StateBuffer_[idx_state_].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_);

  predictProcessCovariance(StateBuffer_[idx_P_].time_ - StateBuffer_[(unsigned char)(idx_P_ - 1)].time_);

  isnumeric = checkForNumeric((double*)(&StateBuffer_[idx_state_ - 1].p_[0]), 3, "prediction p");
  isnumeric = checkForNumeric((double*)(&StateBuffer_[idx_state_ - 1].P_(0)), N_STATE * N_STATE, "prediction done P");

  predictionMade_ = true;

  msgPose_.header.stamp = msg->header.stamp;
  msgPose_.header.seq = msg->header.seq;

  StateBuffer_[(unsigned char)(idx_state_ - 1)].toPoseMsg(msgPose_);
  pubPose_.publish(msgPose_);

  msgPoseCtrl_.header = msgPose_.header;
  StateBuffer_[(unsigned char)(idx_state_ - 1)].toExtStateMsg(msgPoseCtrl_);
  pubPoseCrtl_.publish(msgPoseCtrl_);

  seq++;
}


void MSF_Core::propagateState(const double dt)
{
  typedef const Eigen::Matrix<double, 4, 4> ConstMatrix4;
  typedef const Eigen::Matrix<double, 3, 1> ConstVector3;
  typedef Eigen::Matrix<double, 4, 4> Matrix4;

  // get references to current and previous state
  State & cur_state = StateBuffer_[idx_state_];
  State & prev_state = StateBuffer_[(unsigned char)(idx_state_ - 1)];

  // zero props:
  cur_state.b_w_ = prev_state.b_w_;
  cur_state.b_a_ = prev_state.b_a_;
  cur_state.L_ = prev_state.L_;
  cur_state.q_wv_ = prev_state.q_wv_;
  cur_state.q_ci_ = prev_state.q_ci_;
  cur_state.p_ci_ = prev_state.p_ci_;

//  Eigen::Quaternion<double> dq;
  Eigen::Matrix<double, 3, 1> dv;
  ConstVector3 ew = cur_state.w_m_ - cur_state.b_w_;
  ConstVector3 ewold = prev_state.w_m_ - prev_state.b_w_;
  ConstVector3 ea = cur_state.a_m_ - cur_state.b_a_;
  ConstVector3 eaold = prev_state.a_m_ - prev_state.b_a_;
  ConstMatrix4 Omega = omegaMatJPL(ew);
  ConstMatrix4 OmegaOld = omegaMatJPL(ewold);
  Matrix4 OmegaMean = omegaMatJPL((ew + ewold) / 2);

  // zero order quaternion integration
  //	cur_state.q_ = (Eigen::Matrix<double,4,4>::Identity() + 0.5*Omega*dt)*StateBuffer_[(unsigned char)(idx_state_-1)].q_.coeffs();

  // first order quaternion integration, this is kind of costly and may not add a lot to the quality of propagation...
  int div = 1;
  Matrix4 MatExp;
  MatExp.setIdentity();
  OmegaMean *= 0.5 * dt;
  for (int i = 1; i < 5; i++)
  {
    div *= i;
    MatExp = MatExp + OmegaMean / div;
    OmegaMean *= OmegaMean;
  }

  // first oder quat integration matrix
  ConstMatrix4 quat_int = MatExp + 1.0 / 48.0 * (Omega * OmegaOld - OmegaOld * Omega) * dt * dt;

  // first oder quaternion integration
  cur_state.q_.coeffs() = quat_int * prev_state.q_.coeffs();
  cur_state.q_.normalize();

  // first oder quaternion integration
  cur_state.q_int_.coeffs() = quat_int * prev_state.q_int_.coeffs();
  cur_state.q_int_.normalize();

  dv = (cur_state.q_.toRotationMatrix() * ea + prev_state.q_.toRotationMatrix() * eaold) / 2;
  cur_state.v_ = prev_state.v_ + (dv - g_) * dt;
  cur_state.p_ = prev_state.p_ + ((cur_state.v_ + prev_state.v_) / 2 * dt);
  idx_state_++;
}


void MSF_Core::predictProcessCovariance(const double dt)
{

  typedef const Eigen::Matrix<double, 3, 3> ConstMatrix3;
  typedef const Eigen::Matrix<double, 3, 1> ConstVector3;
  typedef Eigen::Vector3d Vector3;

  // noises
  ConstVector3 nav = Vector3::Constant(config_.noise_acc /* / sqrt(dt) */);
  ConstVector3 nbav = Vector3::Constant(config_.noise_accbias /* * sqrt(dt) */);

  ConstVector3 nwv = Vector3::Constant(config_.noise_gyr /* / sqrt(dt) */);
  ConstVector3 nbwv = Vector3::Constant(config_.noise_gyrbias /* * sqrt(dt) */);

  ConstVector3 nqwvv = Eigen::Vector3d::Constant(config_.noise_qwv);
  ConstVector3 nqciv = Eigen::Vector3d::Constant(config_.noise_qci);
  ConstVector3 npicv = Eigen::Vector3d::Constant(config_.noise_pic);

  // bias corrected IMU readings
  ConstVector3 ew = StateBuffer_[idx_P_].w_m_ - StateBuffer_[idx_P_].b_w_;
  ConstVector3 ea = StateBuffer_[idx_P_].a_m_ - StateBuffer_[idx_P_].b_a_;

  ConstMatrix3 a_sk = skew(ea);
  ConstMatrix3 w_sk = skew(ew);
  ConstMatrix3 eye3 = Eigen::Matrix<double, 3, 3>::Identity();

  ConstMatrix3 C_eq = StateBuffer_[idx_P_].q_.toRotationMatrix();

  const double dt_p2_2 = dt * dt * 0.5; // dt^2 / 2
  const double dt_p3_6 = dt_p2_2 * dt / 3.0; // dt^3 / 6
  const double dt_p4_24 = dt_p3_6 * dt * 0.25; // dt^4 / 24
  const double dt_p5_120 = dt_p4_24 * dt * 0.2; // dt^5 / 120

  ConstMatrix3 Ca3 = C_eq * a_sk;
  ConstMatrix3 A = Ca3 * (-dt_p2_2 * eye3 + dt_p3_6 * w_sk - dt_p4_24 * w_sk * w_sk);
  ConstMatrix3 B = Ca3 * (dt_p3_6 * eye3 - dt_p4_24 * w_sk + dt_p5_120 * w_sk * w_sk);
  ConstMatrix3 D = -A;
  ConstMatrix3 E = eye3 - dt * w_sk + dt_p2_2 * w_sk * w_sk;
  ConstMatrix3 F = -dt * eye3 + dt_p2_2 * w_sk - dt_p3_6 * (w_sk * w_sk);
  ConstMatrix3 C = Ca3 * F;

  // discrete error state propagation Matrix Fd according to:
  // Stephan Weiss and Roland Siegwart.
  // Real-Time Metric State Estimation for Modular Vision-Inertial Systems.
  // IEEE International Conference on Robotics and Automation. Shanghai, China, 2011
  Fd_.setIdentity();
  Fd_.block<3, 3> (0, 3) = dt * eye3;
  Fd_.block<3, 3> (0, 6) = A;
  Fd_.block<3, 3> (0, 9) = B;
  Fd_.block<3, 3> (0, 12) = -C_eq * dt_p2_2;

  Fd_.block<3, 3> (3, 6) = C;
  Fd_.block<3, 3> (3, 9) = D;
  Fd_.block<3, 3> (3, 12) = -C_eq * dt;

  Fd_.block<3, 3> (6, 6) = E;
  Fd_.block<3, 3> (6, 9) = F;

  calc_Q(dt, StateBuffer_[idx_P_].q_, ew, ea, nav, nbav, nwv, nbwv, config_.noise_scale, nqwvv, nqciv, npicv, Qd_);
  StateBuffer_[idx_P_].P_ = Fd_ * StateBuffer_[(unsigned char)(idx_P_ - 1)].P_ * Fd_.transpose() + Qd_;

  idx_P_++;
}


bool MSF_Core::getStateAtIdx(State* timestate, unsigned char idx)
{
  if (!predictionMade_)
  {
    timestate->time_ = -1;
    return false;
  }

  *timestate = StateBuffer_[idx];

  return true;
}

unsigned char MSF_Core::getClosestState(State* timestate, ros::Time tstamp, double delay)
{
  if (!predictionMade_)
  {
    timestate->time_ = -1;
    return false;
  }

  unsigned char idx = (unsigned char)(idx_state_ - 1);
  double timedist = 1e100;
  double timenow = tstamp.toSec() - delay - config_.delay;

  while (fabs(timenow - StateBuffer_[idx].time_) < timedist) // timedist decreases continuously until best point reached... then rises again
  {
    timedist = fabs(timenow - StateBuffer_[idx].time_);
    idx--;
  }
  idx++; // we subtracted one too much before....

  static bool started = false;
  if (idx == 1 && !started)
    idx = 2;
  started = true;

  if (StateBuffer_[idx].time_ == 0)
    return false; // // early abort // //  not enough predictions made yet to apply measurement (too far in past)

  propPToIdx(idx); // catch up with covariance propagation if necessary

  *timestate = StateBuffer_[idx];

  return idx;
}

void MSF_Core::propPToIdx(unsigned char idx)
{
  // propagate cov matrix until idx
  if (idx<idx_state_ && (idx_P_<=idx || idx_P_>idx_state_))	//need to propagate some covs
	  while (idx!=(unsigned char)(idx_P_-1))
		  predictProcessCovariance(StateBuffer_[idx_P_].time_-StateBuffer_[(unsigned char)(idx_P_-1)].time_);
}

bool MSF_Core::applyCorrection(unsigned char idx_delaystate, const ErrorState & res_delayed, double fuzzythres)
{
  static int seq_m = 0;
  if (config_.fixed_scale)
  {
    correction_(15) = 0; //scale
  }

  if (config_.fixed_bias)
  {
    correction_(9) = 0; //acc bias x
    correction_(10) = 0; //acc bias y
    correction_(11) = 0; //acc bias z
    correction_(12) = 0; //gyro bias x
    correction_(13) = 0; //gyro bias y
    correction_(14) = 0; //gyro bias z
  }

  if (config_.fixed_calib)
  {
    correction_(19) = 0; //q_ic roll
    correction_(20) = 0; //q_ic pitch
    correction_(21) = 0; //q_ic yaw
    correction_(22) = 0; //p_ci x
    correction_(23) = 0; //p_ci y
    correction_(24) = 0; //p_ci z
  }

  // state update:

  // store old values in case of fuzzy tracking
  // TODO: what to do with attitude? augment measurement noise?

  State & delaystate = StateBuffer_[idx_delaystate];

  const Eigen::Matrix<double, 3, 1> buff_bw = delaystate.b_w_;
  const Eigen::Matrix<double, 3, 1> buff_ba = delaystate.b_a_;
  const double buff_L = delaystate.L_;
  const Eigen::Quaternion<double> buff_qwv = delaystate.q_wv_;
  const Eigen::Quaternion<double> buff_qci = delaystate.q_ci_;
  const Eigen::Matrix<double, 3, 1> buff_pic = delaystate.p_ci_;

  delaystate.p_ = delaystate.p_ + correction_.block<3, 1> (0, 0);
  delaystate.v_ = delaystate.v_ + correction_.block<3, 1> (3, 0);
  delaystate.b_w_ = delaystate.b_w_ + correction_.block<3, 1> (9, 0);
  delaystate.b_a_ = delaystate.b_a_ + correction_.block<3, 1> (12, 0);
  delaystate.L_ = delaystate.L_ + correction_(15);
  if (delaystate.L_ < 0)
  {
    ROS_WARN_STREAM_THROTTLE(1,"Negative scale detected: " << delaystate.L_ << ". Correcting to 0.1");
    delaystate.L_ = 0.1;
  }

  Eigen::Quaternion<double> qbuff_q = quaternionFromSmallAngle(correction_.block<3, 1> (6, 0));
  delaystate.q_ = delaystate.q_ * qbuff_q;
  delaystate.q_.normalize();

  Eigen::Quaternion<double> qbuff_qwv = quaternionFromSmallAngle(correction_.block<3, 1> (16, 0));
  delaystate.q_wv_ = delaystate.q_wv_ * qbuff_qwv;
  delaystate.q_wv_.normalize();

  Eigen::Quaternion<double> qbuff_qci = quaternionFromSmallAngle(correction_.block<3, 1> (19, 0));
  delaystate.q_ci_ = delaystate.q_ci_ * qbuff_qci;
  delaystate.q_ci_.normalize();

  delaystate.p_ci_ = delaystate.p_ci_ + correction_.block<3, 1> (22, 0);

	// update qbuff_ and check for fuzzy tracking
  if (qvw_inittimer_ > nBuff_)
  {
    // should be unit quaternion if no error
    Eigen::Quaternion<double> errq = delaystate.q_wv_.conjugate() *
        Eigen::Quaternion<double>(
            getMedian(qbuff_.block<nBuff_, 1> (0, 3)),
            getMedian(qbuff_.block<nBuff_, 1> (0, 0)),
            getMedian(qbuff_.block<nBuff_, 1> (0, 1)),
            getMedian(qbuff_.block<nBuff_, 1> (0, 2))
            );

    if (std::max(errq.vec().maxCoeff(), -errq.vec().minCoeff()) / fabs(errq.w()) * 2 > fuzzythres) // fuzzy tracking (small angle approx)
    {
      ROS_WARN_STREAM_THROTTLE(1,"fuzzy tracking triggered: " << std::max(errq.vec().maxCoeff(), -errq.vec().minCoeff())/fabs(errq.w())*2 << " limit: " << fuzzythres <<"\n");

      //state_.q_ = buff_q;
      delaystate.b_w_ = buff_bw;
      delaystate.b_a_ = buff_ba;
      delaystate.L_ = buff_L;
      delaystate.q_wv_ = buff_qwv;
      delaystate.q_ci_ = buff_qci;
      delaystate.p_ci_ = buff_pic;
      correction_.block<16, 1> (9, 0) = Eigen::Matrix<double, 16, 1>::Zero();
      qbuff_q.setIdentity();
      qbuff_qwv.setIdentity();
      qbuff_qci.setIdentity();
    }
    else // if tracking ok: update mean and 3sigma of past N q_vw's
    {
      qbuff_.block<1, 4> (qvw_inittimer_ - nBuff_ - 1, 0) = Eigen::Matrix<double, 1, 4>(delaystate.q_wv_.coeffs());
      qvw_inittimer_ = (qvw_inittimer_) % nBuff_ + nBuff_ + 1;
    }
  }
  else // at beginning get mean and 3sigma of past N q_vw's
  {
    qbuff_.block<1, 4> (qvw_inittimer_ - 1, 0) = Eigen::Matrix<double, 1, 4>(delaystate.q_wv_.coeffs());
    qvw_inittimer_++;
  }

  // idx fiddeling to ensure correct update until now from the past
  idx_time_ = idx_state_;
  idx_state_ = idx_delaystate + 1;
  idx_P_ = idx_delaystate + 1;

  // propagate state matrix until now
  while (idx_state_ != idx_time_)
    propagateState(StateBuffer_[idx_state_].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_);

  checkForNumeric(&correction_[0], HLI_EKF_STATE_SIZE, "update");

  // publish correction for external propagation
  msgCorrect_.header.stamp = ros::Time::now();
  msgCorrect_.header.seq = seq_m;
  msgCorrect_.angular_velocity.x = 0;
  msgCorrect_.angular_velocity.y = 0;
  msgCorrect_.angular_velocity.z = 0;
  msgCorrect_.linear_acceleration.x = 0;
  msgCorrect_.linear_acceleration.y = 0;
  msgCorrect_.linear_acceleration.z = 0;

  const unsigned char idx = (unsigned char)(idx_state_ - 1);
  msgCorrect_.state[0] = StateBuffer_[idx].p_[0] - hl_state_buf_.state[0];
  msgCorrect_.state[1] = StateBuffer_[idx].p_[1] - hl_state_buf_.state[1];
  msgCorrect_.state[2] = StateBuffer_[idx].p_[2] - hl_state_buf_.state[2];
  msgCorrect_.state[3] = StateBuffer_[idx].v_[0] - hl_state_buf_.state[3];
  msgCorrect_.state[4] = StateBuffer_[idx].v_[1] - hl_state_buf_.state[4];
  msgCorrect_.state[5] = StateBuffer_[idx].v_[2] - hl_state_buf_.state[5];

  Eigen::Quaterniond hl_q(hl_state_buf_.state[6], hl_state_buf_.state[7], hl_state_buf_.state[8], hl_state_buf_.state[9]);
  qbuff_q = hl_q.inverse() * StateBuffer_[idx].q_;
  msgCorrect_.state[6] = qbuff_q.w();
  msgCorrect_.state[7] = qbuff_q.x();
  msgCorrect_.state[8] = qbuff_q.y();
  msgCorrect_.state[9] = qbuff_q.z();

  msgCorrect_.state[10] = StateBuffer_[idx].b_w_[0] - hl_state_buf_.state[10];
  msgCorrect_.state[11] = StateBuffer_[idx].b_w_[1] - hl_state_buf_.state[11];
  msgCorrect_.state[12] = StateBuffer_[idx].b_w_[2] - hl_state_buf_.state[12];
  msgCorrect_.state[13] = StateBuffer_[idx].b_a_[0] - hl_state_buf_.state[13];
  msgCorrect_.state[14] = StateBuffer_[idx].b_a_[1] - hl_state_buf_.state[14];
  msgCorrect_.state[15] = StateBuffer_[idx].b_a_[2] - hl_state_buf_.state[15];

  msgCorrect_.flag = sensor_fusion_comm::ExtEkf::state_correction;
  pubCorrect_.publish(msgCorrect_);

  // publish state
  msgState_.header = msgCorrect_.header;
  StateBuffer_[idx].toStateMsg(msgState_);
  pubState_.publish(msgState_);
  seq_m++;

  return 1;
}

void MSF_Core::Config(msf_core::MSF_CoreConfig& config, uint32_t level)
{
  for (std::vector<CallbackType>::iterator it = callbacks_.begin(); it != callbacks_.end(); it++)
    (*it)(config, level);
}

void MSF_Core::DynConfig(msf_core::MSF_CoreConfig& config, uint32_t level)
{
  config_ = config;
}

double MSF_Core::getMedian(const Eigen::Matrix<double, nBuff_, 1> & data)
{
  std::vector<double> mediandistvec;
  mediandistvec.reserve(nBuff_);
  for (int i = 0; i < nBuff_; ++i)
    mediandistvec.push_back(data(i));

  if (mediandistvec.size() > 0)
  {
    std::vector<double>::iterator first = mediandistvec.begin();
    std::vector<double>::iterator last = mediandistvec.end();
    std::vector<double>::iterator middle = first + std::floor((last - first) / 2);
    std::nth_element(first, middle, last); // can specify comparator as optional 4th arg
    return *middle;
  }
  else
    return 0;
}

}; // end namespace

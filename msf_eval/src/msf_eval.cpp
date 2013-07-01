/*
 * msf eval adapted from aslam_eval by stefan leutenegger
 *
 */

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include "rosbag/bag.h"
#include "rosbag/chunked_file.h"
#include "rosbag/view.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <set>
#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <sm/kinematics/transformations.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <ros/package.h>

Eigen::Matrix<double, 3, 1> getPosition(geometry_msgs::TransformStampedConstPtr tf){
  Eigen::Matrix<double, 3, 1> pos;
  pos << tf->transform.translation.x, tf->transform.translation.y, tf->transform.translation.z;
  return pos;
}

Eigen::Matrix<double, 3, 1> getPosition(geometry_msgs::PoseWithCovarianceStampedConstPtr ps){
  Eigen::Matrix<double, 3, 1> pos;
  pos << ps->pose.pose.position.x, ps->pose.pose.position.y, ps->pose.pose.position.z;
  return pos;
}

Eigen::Quaterniond getOrientation(geometry_msgs::TransformStampedConstPtr tf){
  Eigen::Quaterniond orientation;
  orientation.w() = tf->transform.rotation.w;
  orientation.x() = tf->transform.rotation.x;
  orientation.y() = tf->transform.rotation.y;
  orientation.z() = tf->transform.rotation.z;
  return orientation;
}

Eigen::Quaterniond getOrientation(geometry_msgs::PoseWithCovarianceStampedConstPtr ps){
  Eigen::Quaterniond orientation;
  orientation.w() = ps->pose.pose.orientation.w;
  orientation.x() = ps->pose.pose.orientation.x;
  orientation.y() = ps->pose.pose.orientation.y;
  orientation.z() = ps->pose.pose.orientation.z;
  return orientation;
}

Eigen::Matrix<double, 6, 6> getCovariance(geometry_msgs::PoseWithCovarianceStampedConstPtr ps){
  return Eigen::Matrix<double, 6, 6>(&ps->pose.covariance[0]);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "msf_eval");
  ros::Time::init();


  enum argIndices{
    bagfile = 1,
    EVAL_topic = 2,
    GT_topic = 3,
    singleRunOnly = 4
  };

  //calibration of sensor to vicon
  Eigen::Matrix4d T_BaBg_mat;

  //this is the Vicon one - SLAM sensor V0
  ///////////////////////////////////////////////////////////////////////////////////////////
  T_BaBg_mat << 0.999706627053000, -0.022330158354000, 0.005123243528000, -0.060614697387000, 0.022650462142000, 0.997389634278000, -0.068267398302000, 0.035557942651000, -0.003589706237000, 0.068397960288000, 0.997617159323000, -0.042589657349000, 0, 0, 0, 1.000000000000000;
  ////////////////////////////////////////////////////////////////////////////////////////////

  ros::Duration dt(0.0039);
  const double timeSyncThreshold = 0.005;
  const double timeDiscretization = 10.0; //discretization step for different starting points into the dataset
  const double trajectoryTimeDiscretization = 0.049; //discretization of evaluation points (vision framerate for fair comparison)
  const double startTimeOffset = 10.0;

  if (argc < 4)
  {
    MSF_ERROR_STREAM("usage: ./"<<argv[0]<<" bagfile EVAL_topic GT_topic [singleRunOnly]");
    return -1;
  }
  bool singleRun = false;
  if (argc == 5)
  {
    singleRun = atoi(argv[singleRunOnly]);
  }

  if(singleRun){
    MSF_WARN_STREAM("Doing only a single run.");
  }else{
    MSF_WARN_STREAM("Will process the dataset from different starting points.");
  }

  typedef geometry_msgs::TransformStamped GT_TYPE;
  typedef geometry_msgs::PoseWithCovarianceStamped EVAL_TYPE;

  // output file
  std::stringstream matlab_fname;

  std::string path = ros::package::getPath("msf_eval");
  matlab_fname << path << "/Matlab/matlab_data" << ros::Time::now().sec << ".m";

  std::ofstream outfile(matlab_fname.str().c_str());

  std::stringstream poses_EVAL;
  std::stringstream poses_GT;

  assert(outfile.good());

  outfile << "data=[" << std::endl;

  ros::Duration startOffset(startTimeOffset);

  sm::kinematics::Transformation T_BaBg(T_BaBg_mat); //body aslam to body Ground Truth

  // open for reading
  rosbag::Bag bag(argv[bagfile], rosbag::bagmode::Read);

  // views on topics
  rosbag::View view_EVAL(bag, rosbag::TopicQuery(argv[EVAL_topic]));
  rosbag::View view_GT(bag, rosbag::TopicQuery(argv[GT_topic]));

  //check topics
  if (view_EVAL.size() == 0)
  {
    MSF_ERROR_STREAM("The bag you provided does not contain messages for topic "<<argv[EVAL_topic]);
    return -1;
  }
  if (view_GT.size() == 0)
  {
    MSF_ERROR_STREAM("The bag you provided does not contain messages for topic "<<argv[GT_topic]);
    return -1;
  }

  //litter console with number of messages
  MSF_INFO_STREAM("Reading from "<<argv[bagfile]);
  MSF_INFO_STREAM("Topic "<<argv[EVAL_topic]<<", size: "<<view_EVAL.size());
  MSF_INFO_STREAM("Topic "<<argv[GT_topic]<<", size: "<<view_GT.size());

  //get times of first messages
  GT_TYPE::ConstPtr GT_begin = view_GT.begin()->instantiate<GT_TYPE>();
  assert(GT_begin);
  EVAL_TYPE::ConstPtr POSE_begin = view_EVAL.begin()->instantiate<EVAL_TYPE>();
  assert(POSE_begin);

  MSF_INFO_STREAM("First GT data at "<<GT_begin->header.stamp);
  MSF_INFO_STREAM("First EVAL data at "<<POSE_begin->header.stamp);

  while (true) // start eval from different starting points
  {

    rosbag::View::const_iterator it_EVAL = view_EVAL.begin();
    rosbag::View::const_iterator it_GT = view_GT.begin();

    // read ground truth
    ros::Time start;

    // Find start time alignment: set the GT iterater to point to a time larger than the aslam time
    while (true)
    {
      GT_TYPE::ConstPtr trafo = it_GT->instantiate<GT_TYPE>();
      assert(trafo);
      ros::Time time_GT;
      time_GT = trafo->header.stamp + dt;

      EVAL_TYPE::ConstPtr pose = it_EVAL->instantiate<EVAL_TYPE>();
      assert(pose);

      ros::Time time_EKF;
      time_EKF = pose->header.stamp;

      if (time_EKF >= time_GT)
      {
        it_GT++;
        if (it_GT == view_GT.end())
        {
          MSF_ERROR_STREAM("Time synchronization failed");
          return false;
        }
      }
      else
      {
        start = time_GT;
        MSF_INFO_STREAM("Time synced! GT start: "<<start <<" EVAL start: "<<time_EKF);
        break;
      }
    }

    // world frame alignment
    sm::kinematics::Transformation T_WaBa;
    sm::kinematics::Transformation T_WgBg;
    sm::kinematics::Transformation T_WgBg_last;
    sm::kinematics::Transformation T_WaWg;

    // now find the GT/EKF pairings
    int ctr = 0; //how many meas did we add this run?
    double ds = 0.0; // distance travelled
    ros::Time lastTime(0.0);

    MSF_INFO_STREAM("Processing measurements... Current start point: "<<startOffset<<"s into the bag.");

    for (; it_GT != view_GT.end(); ++it_GT)
    {
      GT_TYPE::ConstPtr trafo = it_GT->instantiate<GT_TYPE>();
      assert(trafo);

      // find closest timestamp
      ros::Time time_GT = trafo->header.stamp + dt;

      EVAL_TYPE::ConstPtr pose = it_EVAL->instantiate<EVAL_TYPE>();
      assert(pose);

      ros::Time time_EKF = pose->header.stamp;

      bool terminate = false;
      //get the measurement close to this GT value
      while (time_GT > time_EKF)
      {
        it_EVAL++;
        if (it_EVAL == view_EVAL.end())
        {
          terminate = true;
          MSF_INFO_STREAM("done. All EKF meas processed!");
          break;
        }
        pose = it_EVAL->instantiate<EVAL_TYPE>();
        assert(pose);
        time_EKF = pose->header.stamp;
      }
      if (terminate){
        break;
      }

      // add comparison value
      if (time_GT - start >= startOffset)
      {

        T_WaBa = sm::kinematics::Transformation(
            Eigen::Vector4d(-pose->pose.pose.orientation.x, -pose->pose.pose.orientation.y,
                            -pose->pose.pose.orientation.z, pose->pose.pose.orientation.w),
                            Eigen::Vector3d(pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z));

        T_WgBg = sm::kinematics::Transformation(
            Eigen::Vector4d(-trafo->transform.rotation.x, -trafo->transform.rotation.y, -trafo->transform.rotation.z,
                            trafo->transform.rotation.w),
                            Eigen::Vector3d(trafo->transform.translation.x, trafo->transform.translation.y,
                                            trafo->transform.translation.z));

        // initial alignment
        if (ctr == 0)
        {
          T_WaWg = (T_WaBa) * T_BaBg * T_WgBg.inverse();
          T_WgBg_last = T_WgBg;
        }

        sm::kinematics::Transformation dT = T_WaBa * (T_WaWg * T_WgBg * T_BaBg.inverse()).inverse();
        sm::kinematics::Transformation T_WaBa_gt = (T_WaWg * T_WgBg * T_BaBg.inverse());

        T_WaBa_gt = sm::kinematics::Transformation(T_WaBa_gt.q().normalized(), T_WaBa_gt.t());
        dT = sm::kinematics::Transformation(dT.q().normalized(), dT.t());

        // update integral
        if (trafo)
        {
          ds += (T_WgBg.t() - T_WgBg_last.t()).norm();
          // store last GT transformation
          T_WgBg_last = T_WgBg;
        }
        else
        {
          // too noisy
          if ((T_WgBg * T_WgBg_last.inverse()).t().norm() > .1)
          {
            ds += (T_WgBg.t() - T_WgBg_last.t()).norm();
            //MSF_INFO_STREAM((T_WgBg*T_WgBg_last.inverse()).t().norm());
            // store last GT transformation
            T_WgBg_last = T_WgBg;
          }
        }
        Eigen::Vector3d dalpha;
        if (dT.q().head<3>().norm() > 1e-12)
        {
          dalpha = (asin(dT.q().head<3>().norm()) * 2 * dT.q().head<3>().normalized());
        }
        else
        {
          dalpha = 2 * dT.q().head<3>();
        }
        // g-vector alignment
        Eigen::Vector3d e_z_Wg(0, 0, 1);
        Eigen::Vector3d e_z_Wa = dT.C() * e_z_Wg;
        double dalpha_e_z = acos(std::min(1.0, e_z_Wg.dot(e_z_Wa)));

        if (fabs((time_GT - time_EKF).toSec()) < timeSyncThreshold
            && (time_EKF - lastTime).toSec() > trajectoryTimeDiscretization)
        {
          if (startOffset.toSec() == startTimeOffset)
          {
            poses_EVAL << T_WaBa.t().transpose() << ";" << std::endl;
            poses_GT << T_WgBg.t().transpose() << ";" << std::endl;
          }
          Eigen::Matrix<double, 6, 6> cov = getCovariance(pose);
          outfile << (time_GT - start).toSec() << " "
              << ds << " " << (T_WaBa.t() - T_WaBa_gt.t()).norm() << " " //translation
              << 2 * acos(std::min(1.0, fabs(dT.q()[3]))) << " " << dalpha_e_z << " " //orientation
              <<cov(0, 0)<<" "<<cov(1, 1)<<" "<<cov(2, 2)<<" " //position covariances
              <<cov(3, 3)<<" "<<cov(4, 4)<<" "<<cov(5, 5)<<";" //orientation covariances
              << std::endl;

          lastTime = time_EKF; // remember
        }

        // count comparisons
        ctr++;

      }
    }

    MSF_INFO_STREAM("Added "<<ctr<<" measurement edges.");

    //where in the bag should the next eval start
    startOffset += ros::Duration(timeDiscretization);


    if (ctr == 0 || singleRun) //any new measurements this run?
    {
      break;
    }
  }

  // cleanup
  bag.close();

  outfile << "];";
  outfile << "poses = [" << poses_EVAL.str() << "];" << std::endl;
  outfile << "poses_GT = [" << poses_GT.str() << "];" << std::endl;
  outfile.close();

  return 0;
}

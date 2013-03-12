/*
 * aslam_eval.cpp
 *
 *  Created on: Dez 26, 2012
 *      Author: lestefan
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "msf_eval");

  if (argc != 4)
  {
    ROS_ERROR_STREAM("usage: ./"<<argv[0]<<" bagfile EKF_topic GT_topic");
    return -1;
  }

  enum{
    bagfile = 1,
    EKF_topic = 2,
    GT_topic = 3
  };

  // output file
  std::stringstream matlab_fname;
  ros::Time::init();
  std::string path = ros::package::getPath("msf_eval");
  matlab_fname << path << "/Matlab/matlab_data" << ros::Time::now().sec << ".m";
  std::ofstream outfile(matlab_fname.str().c_str());
  std::stringstream poses_EKF;
  std::stringstream poses_GT;
  assert(outfile.good());
  outfile << "data=[" << std::endl;

  // consts - TODO: get this from calibration or so
  Eigen::Matrix4d T_BaBg_mat;

  //this is the Vicon one - Jörn
  ///////////////////////////////////////////////////////////////////////////////////////////
  T_BaBg_mat << 0.999706627053000, -0.022330158354000, 0.005123243528000, -0.060614697387000, 0.022650462142000, 0.997389634278000, -0.068267398302000, 0.035557942651000, -0.003589706237000, 0.068397960288000, 0.997617159323000, -0.042589657349000, 0, 0, 0, 1.000000000000000;
  ros::Duration dt(0.0039);
  const double timeSyncThreshold = 0.005;
  const double timeDiscretization = 10.0;
  const double trajectoryTimeDiscretization = 0.049;
  const double startTimeOffset = 0.0;
  ros::Duration startOffset(startTimeOffset);
  ////////////////////////////////////////////////////////////////////////////////////////////

  // this is the Vicon one - my estimate
  /*T_BaBg_mat <<
   0.999649, -0.00828671,  -0.0251241,  -0.0756163,
   0.00741642,    0.999375,  -0.0345373,   0.0838595,
   0.0253946,   0.0343388,    0.999087,   -0.129142,
   0,           0,           0,          1;*/
  /*
   0.0328386,   0.042941,   0.998537, -0.0443063,
   0.999378, -0.0142124, -0.0322551,  -0.152243,
   0.0128065,   0.998976,  -0.043381, -0.0585155,
   0,          0,          0,          1;

   0.0276559,  0.0483769,   0.998446, -0.0568853,
   0.999469, -0.0185021, -0.0267878,  -0.179351,
   0.0171775,   0.998657, -0.0488629, -0.0567147,
   0,          0,          0,          1;*/
  /*T_BaBg_mat << 0.0000, 0.0000, 1.0000, 0.0000,
   1.0000, 0.0000, 0.0000, 0.0000,
   0.0000, 1.0000, 0.0000, 0.0000,
   0.0000, 0.0000, 0.0000, 1.0000;*/

  sm::kinematics::Transformation T_BaBg(T_BaBg_mat); //body aslam to body Ground Truth
//  while (true)
//  {

    // open for reading
    rosbag::Bag bag(argv[bagfile], rosbag::bagmode::Read);

    // views on topics
    rosbag::View view_EKF(bag, rosbag::TopicQuery(argv[EKF_topic]));
    rosbag::View view_GT(bag, rosbag::TopicQuery(argv[GT_topic]));

    //check topics
    if (view_EKF.size() == 0)
    {
      ROS_ERROR_STREAM("The bag you provided does not contain messages for topic "<<argv[EKF_topic]);
      return -1;
    }
    if (view_GT.size() == 0)
    {
      ROS_ERROR_STREAM("The bag you provided does not contain messages for topic "<<argv[GT_topic]);
      return -1;
    }

    //litter console with number of messages
    ROS_INFO_STREAM("Reading from "<<argv[bagfile]);
    ROS_INFO_STREAM("Topic "<<argv[EKF_topic]<<", size: "<<view_EKF.size());
    ROS_INFO_STREAM("Topic "<<argv[GT_topic]<<", size: "<<view_GT.size());

    //message iterators
    rosbag::View::const_iterator it_EKF = view_EKF.begin();
    rosbag::View::const_iterator it_GT = view_GT.begin();

    // read ground truth
    ros::Time start; // remember

    // Find start time alignment: set the GT iterater to point to a time larger than the aslam time
    while (true)
    {
      geometry_msgs::TransformStamped::ConstPtr trafo = it_GT->instantiate<geometry_msgs::TransformStamped>();
      assert(trafo);
      ros::Time time_GT;
      time_GT = trafo->header.stamp + dt;

      geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose = it_EKF->instantiate<
          geometry_msgs::PoseWithCovarianceStamped>();
      assert(pose);

      ros::Time time_EKF;
      time_EKF = pose->header.stamp;

      if (time_EKF >= time_GT)
      {
        it_GT++;
        if (it_GT == view_GT.end())
        {
          ROS_ERROR_STREAM("Time synchronization failed");
          return false;
        }
      }
      else
      {
        start = time_GT;
        ROS_INFO_STREAM("Time synced: "<<start);
        break;
      }
    }

    // world frame alignment
    sm::kinematics::Transformation T_WaBa;
    sm::kinematics::Transformation T_WgBg;
    sm::kinematics::Transformation T_WgBg_last;
    sm::kinematics::Transformation T_WaWg;

    // now find the GT/EKF pairings
    int ctr = 0;
    double ds = 0.0; // distance travelled
    ros::Time lastTime(0.0);

    ROS_INFO_STREAM("Processing measurements... ");
    for (; it_GT != view_GT.end(); ++it_GT)
    {
      geometry_msgs::TransformStamped::ConstPtr trafo = it_GT->instantiate<geometry_msgs::TransformStamped>();
      assert(trafo);

      // find closest timestamp
      ros::Time time_GT = trafo->header.stamp + dt;

      geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose = it_EKF->instantiate<
          geometry_msgs::PoseWithCovarianceStamped>();
      assert(pose);

      ros::Time time_EKF = pose->header.stamp;

      bool terminate = false;
      while (time_GT > time_EKF)
      {
        it_EKF++;
        if (it_EKF == view_EKF.end())
        {
          terminate = true;
          ROS_INFO_STREAM("done. All EKF meas processed!");
          break;
        }
        pose = it_EKF->instantiate<geometry_msgs::PoseWithCovarianceStamped>();
        assert(pose);
        time_EKF = pose->header.stamp;
      }
      if (terminate)
        break;

      // add measurement edge
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
            //ROS_INFO_STREAM((T_WgBg*T_WgBg_last.inverse()).t().norm());
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
        //ROS_ERROR_STREAM("dalpha_e_z="<<dalpha_e_z<<"; 2*acos(std::min(1.0,fabs(dT.q()[3])))="<<2*acos(std::min(1.0,fabs(dT.q()[3]))));
        //assert(dalpha_e_z<=2*acos(std::min(1.0,fabs(dT.q()[3]))));
        //ROS_ERROR_STREAM((time_GT-time_aslam).toSec());
        if (fabs((time_GT - time_EKF).toSec()) < timeSyncThreshold
            && (time_EKF - lastTime).toSec() > trajectoryTimeDiscretization)
        {
          if (startOffset.toSec() == startTimeOffset)
          {
            poses_EKF << T_WaBa.t().transpose() << ";" << std::endl;
            poses_GT << T_WgBg.t().transpose() << ";" << std::endl;
          }
          outfile << (time_GT - start).toSec() << " " << ds << " " << (T_WaBa.t() - T_WaBa_gt.t()).norm() << " "
              << 2 * acos(std::min(1.0, fabs(dT.q()[3]))) << " " << dalpha_e_z << ";" << std::endl;
//          ROS_WARN_STREAM(
//              (time_GT-time_EKF).toSec()<<" "<<(time_GT-start).toSec()<<" "<<ds<<" "<<(T_WaBa.t()-T_WaBa_gt.t()).norm()<<" "<<2*acos(std::min(1.0,fabs(dT.q()[3])))<<" "<<dalpha_e_z<<" time="<<time_GT);

          lastTime = time_EKF; // remember
        }
        //ROS_INFO_STREAM((time_GT-time_aslam).toSec()<<" "<<(time_GT-start).toSec()<<" "<<ds<<" "<<dT.t().norm()<<" "<<" "<<2*acos(std::min(1.0,fabs(dT.q()[3]))));

        // count comparisons
        ctr++;

      }
    }

    ROS_INFO_STREAM("Added "<<ctr<<" measurement edges.");

    // check if finished
    startOffset += ros::Duration(timeDiscretization);

    // cleanup
    bag.close();

//    if (ctr == 0)
//    {
//      break;
//    }
//  }

  outfile << "];";
  outfile << "poses = [" << poses_EKF.str() << "];" << std::endl;
  outfile << "poses_GT = [" << poses_GT.str() << "];" << std::endl;
  outfile.close();

  return 0;
}

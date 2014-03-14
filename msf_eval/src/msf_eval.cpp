/*
 * msf eval adapted from aslam_eval by stefan leutenegger
 *
 */

#include <boost/thread.hpp>
#include <Eigen/SVD>
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
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sm/kinematics/transformations.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <ros/package.h>
#include <msf_core/msf_macros.h>

Eigen::Matrix<double, 3, 1> getPosition(
    geometry_msgs::TransformStampedConstPtr tf) {
  Eigen::Matrix<double, 3, 1> pos;
  pos << tf->transform.translation.x, tf->transform.translation.y, tf->transform
      .translation.z;
  return pos;
}

Eigen::Matrix<double, 3, 1> getPosition(
    geometry_msgs::PoseWithCovarianceStampedConstPtr ps) {
  Eigen::Matrix<double, 3, 1> pos;
  pos << ps->pose.pose.position.x, ps->pose.pose.position.y, ps->pose.pose
      .position.z;
  return pos;
}

Eigen::Quaterniond getOrientation(geometry_msgs::TransformStampedConstPtr tf) {
  Eigen::Quaterniond orientation;
  orientation.w() = tf->transform.rotation.w;
  orientation.x() = tf->transform.rotation.x;
  orientation.y() = tf->transform.rotation.y;
  orientation.z() = tf->transform.rotation.z;
  return orientation;
}

Eigen::Quaterniond getOrientation(
    geometry_msgs::PoseWithCovarianceStampedConstPtr ps) {
  Eigen::Quaterniond orientation;
  orientation.w() = ps->pose.pose.orientation.w;
  orientation.x() = ps->pose.pose.orientation.x;
  orientation.y() = ps->pose.pose.orientation.y;
  orientation.z() = ps->pose.pose.orientation.z;
  return orientation;
}

Eigen::Matrix<double, 6, 6> getCovariance(
    geometry_msgs::PoseWithCovarianceStampedConstPtr ps) {
  return Eigen::Matrix<double, 6, 6>(&ps->pose.covariance[0]);
}

void EstimateRigidTransform(
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& points1,
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& points2,
    sm::kinematics::Transformation* transformation) {
  assert(points1.cols() == points2.cols());
  Eigen::Matrix<double, 3, 1> points1m;
  Eigen::Matrix<double, 3, 1> points2m;
  points1m.setZero();
  points2m.setZero();

  for (int col = 0; col < points1.cols(); ++col) {
    points1m += points1.block<3, 1>(0, col);
    points2m += points2.block<3, 1>(0, col);
  }
  points1m /= points1.cols();
  points2m /= points2.cols();

  Eigen::Matrix<double, 3, Eigen::Dynamic> points1_c = points1;
  Eigen::Matrix<double, 3, Eigen::Dynamic> points2_c = points2;

  for (int col = 0; col < points1_c.cols(); ++col) {
    points1_c.block<3, 1>(0, col) -= points1m;
    points2_c.block<3, 1>(0, col) -= points2m;
  }

  // Calculate best rotation using SVD.
  Eigen::Matrix<double, 3, 3> points1Tpoints2 = points1_c
      * points2_c.transpose();

  Eigen::JacobiSVD < Eigen::MatrixXd
      > svd(points1Tpoints2, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<double, 3, 3> U = svd.matrixU();
  Eigen::Matrix<double, 3, 3> V = svd.matrixV();
  //TODO(slynen) fix this inversion.
  U.block<3, 1>(0, 0) = -U.block<3, 1>(0, 0);
  V.block<3, 1>(0, 0) = -V.block<3, 1>(0, 0);

  // Check that the solution is not a reflection:
  Eigen::Matrix<double, 3, 3> reflect;
  reflect.setIdentity();
  reflect(2, 2) = (U * V.transpose()).determinant();
  Eigen::Matrix<double, 3, 3> R = (U * reflect * V.transpose()).transpose();

  // Solve for the translation vector.
  Eigen::Matrix<double, 3, 1> translation = points2m - R * points1m;
  Eigen::Matrix<double, 4, 1> rotation = sm::kinematics::r2quat(R);

  Eigen::Matrix<double, 1, Eigen::Dynamic> ones;
  ones.resize(Eigen::NoChange, points1.cols());
  ones.setConstant(1.0);

  Eigen::Matrix<double, 3, Eigen::Dynamic> point2f = R * points1
      + translation * ones;
  Eigen::Matrix<double, 3, Eigen::Dynamic> dpoint2 = points2 - point2f;

  std::cout << "Alignment error " << dpoint2.norm() << std::endl;
  assert(transformation);
  *transformation = sm::kinematics::Transformation(rotation, translation);
}

#define VICON

int main(int argc, char **argv) {
  ros::init(argc, argv, "msf_eval");
  ros::Time::init();

  enum argIndices {
    bagfile = 1,
    EVAL_topic = 2,
    GT_topic = 3,
    singleRunOnly = 4
  };

  //this is the Vicon one - SLAM sensor V0
  ///////////////////////////////////////////////////////////////////////////////////////////
  //Eigen::Matrix4d T_BaBg_mat;
  //T_BaBg_mat << 0.999706627053000, -0.022330158354000, 0.005123243528000, -0.060614697387000,
  //              0.022650462142000, 0.997389634278000, -0.068267398302000, 0.035557942651000,
  //             -0.003589706237000, 0.068397960288000, 0.997617159323000, -0.042589657349000,
  //              0, 0, 0, 1.000000000000000;
  ////////////////////////////////////////////////////////////////////////////////////////////

  //this is the Vicon one - SLAM sensor V3 - large Rig for Leica eval
  ///////////////////////////////////////////////////////////////////////////////////////////
#ifdef VICON
  Eigen::Matrix4d T_BgBa_mat;
  T_BgBa_mat << 0.167956, 0.95149842, -0.25776255, 0.17104256,
                0.18648779, 0.22608787, 0.95608921, 0.10819705,
                0.96799436, -0.20865049, -0.13947003, 0.09269324,
                0., 0., 0., 1.;
#endif
  ////////////////////////////////////////////////////////////////////////////////////////////

  //this is the Leica one - SLAM sensor V3 - large Rig for Leica eval
    ///////////////////////////////////////////////////////////////////////////////////////////
#ifdef LEICA
//    Eigen::Matrix4d T_BgBa_mat;
//    T_BgBa_mat <<  1, 0, 0, 0.05354061,
//         0, 1, 0, -0.06776346,
//         0, 0, 1, -0.05771415,
//         0, 0, 0, 1;
    ////////////////////////////////////////////////////////////////////////////////////////////
#endif

  sm::kinematics::Transformation T_BgBa(T_BgBa_mat);  // Ground Truth to body

  sm::kinematics::Transformation T_BaBg = T_BgBa.inverse();

//  ros::Duration dt(0.0039);
#ifdef VICON
  ros::Duration dt(-0.010310717511189348);  // Joern Vicon to IMU
#endif
#ifdef LEICA
//  ros::Duration dt( -0.193203053521053);  // Joern Leica to IMU
#endif
  const double timeSyncThreshold = 0.005;
  const double timeDiscretization = 10.0;  //discretization step for different starting points into the dataset
  const double trajectoryTimeDiscretization = 0.049;  //discretization of evaluation points (vision framerate for fair comparison)
  const double startTimeOffset = 10;

  if (argc < 4) {
    MSF_ERROR_STREAM(
        "usage: ./"<<argv[0]<<" bagfile EVAL_topic GT_topic [singleRunOnly]");
    return -1;
  }
  bool singleRun = false;
  if (argc == 5) {
    singleRun = atoi(argv[singleRunOnly]);
  }

  if (singleRun) {
    MSF_WARN_STREAM("Doing only a single run.");
  } else {
    MSF_WARN_STREAM("Will process the dataset from different starting points.");
  }

#ifdef LEICA
  typedef geometry_msgs::PointStamped GT_TYPE;
#endif
#ifdef VICON
  typedef geometry_msgs::TransformStamped GT_TYPE;
#endif
  typedef geometry_msgs::TransformStamped EVAL_TYPE;

  // output file
  std::stringstream matlab_fname;
  std::stringstream matlab_fname_integrate;

  std::string path = ros::package::getPath("msf_eval");
  matlab_fname << path << "/Matlab/matlab_data" << ros::Time::now().sec << ".m";

  std::ofstream outfile(matlab_fname.str().c_str());

  std::stringstream poses_EVAL;
  std::stringstream poses_GT;

  assert(outfile.good());

  outfile << "data=[" << std::endl;

  ros::Duration startOffset(startTimeOffset);

  // open for reading
  rosbag::Bag bag(argv[bagfile], rosbag::bagmode::Read);

  // views on topics
  rosbag::View view_EVAL(bag, rosbag::TopicQuery(argv[EVAL_topic]));
  rosbag::View view_GT(bag, rosbag::TopicQuery(argv[GT_topic]));

  //check topics
  if (view_EVAL.size() == 0) {
    MSF_ERROR_STREAM(
        "The bag you provided does not contain messages for topic "<<argv[EVAL_topic]);
    return -1;
  }
  if (view_GT.size() == 0) {
    MSF_ERROR_STREAM(
        "The bag you provided does not contain messages for topic "<<argv[GT_topic]);
    return -1;
  }

  //litter console with number of messages
  MSF_INFO_STREAM("Reading from "<<argv[bagfile]);
  MSF_INFO_STREAM("Topic "<<argv[EVAL_topic]<<", size: "<<view_EVAL.size());
  MSF_INFO_STREAM("Topic "<<argv[GT_topic]<<", size: "<<view_GT.size());

  //get times of first messages
  GT_TYPE::ConstPtr GT_begin = view_GT.begin()->instantiate<GT_TYPE>();
  if (!GT_begin) {
    MSF_ERROR_STREAM(
        "The bag you provided does not contain messages of the type "
        " which was specified as groundtruth message type");
    return -1;
  }

  EVAL_TYPE::ConstPtr POSE_begin = view_EVAL.begin()->instantiate<EVAL_TYPE>();
  if (!POSE_begin) {
    MSF_ERROR_STREAM(
        "The bag you provided does not contain messages of the type "
        " which was specified as estimator message type");
    return -1;
  }

  MSF_INFO_STREAM("First GT data at "<<GT_begin->header.stamp);
  MSF_INFO_STREAM("First EVAL data at "<<POSE_begin->header.stamp);

  // Estimate transform between frames of reference of gt and estimator.
  sm::kinematics::Transformation T_WaWg_range;
  bool first = true;

  while (true) {  // start eval from different starting points
    rosbag::View::const_iterator it_EVAL = view_EVAL.begin();
    rosbag::View::const_iterator it_GT = view_GT.begin();

    // read ground truth
    ros::Time start;

    // Find start time alignment: set the GT iterator to point to a time larger than the aslam time
    while (true) {
      GT_TYPE::ConstPtr trafo = it_GT->instantiate<GT_TYPE>();
      assert(trafo);
      ros::Time time_GT;
      time_GT = trafo->header.stamp + dt;

      EVAL_TYPE::ConstPtr pose = it_EVAL->instantiate<EVAL_TYPE>();
      assert(pose);

      ros::Time time_EKF;
      time_EKF = pose->header.stamp;

      if (time_EKF >= time_GT) {
        it_GT++;
        if (it_GT == view_GT.end()) {
          MSF_ERROR_STREAM("Time synchronization failed");
          return false;
        }
      } else {
        start = time_GT;
        MSF_INFO_STREAM(
            "Time synced! GT start: "<<start <<" EVAL start: "<<time_EKF);
        break;
      }
    }

    // world frame alignment
    sm::kinematics::Transformation T_WaBa;
    sm::kinematics::Transformation T_WgBg;
    sm::kinematics::Transformation T_WgBg_last;
    sm::kinematics::Transformation T_WaWg;

    if (first) {
      Eigen::Matrix<double, 3, Eigen::Dynamic> points1;
      Eigen::Matrix<double, 3, Eigen::Dynamic> points2;
      double alignment_range = 20;
      double ds = 0.0;

      rosbag::View::const_iterator it_EVAL_align = it_EVAL;
      rosbag::View::const_iterator it_GT_align = it_GT;

      std::vector < Eigen::Matrix<double, 3, 1> > points1_list;
      std::vector < Eigen::Matrix<double, 3, 1> > points2_list;
      std::vector<double> ds_list;

      for (; it_GT_align != view_GT.end(); ++it_GT_align) {
        GT_TYPE::ConstPtr trafo = it_GT_align->instantiate<GT_TYPE>();
        assert(trafo);

        // Find closest timestamp.
        ros::Time time_GT = trafo->header.stamp + dt;

        EVAL_TYPE::ConstPtr pose = it_EVAL_align->instantiate<EVAL_TYPE>();
        ros::Time time_EKF = pose->header.stamp;

        bool terminate = false;
        while (time_GT > time_EKF) {
          it_EVAL_align++;
          if (it_EVAL_align == view_EVAL.end()) {
            terminate = true;
            break;
          }
          pose = it_EVAL_align->instantiate<EVAL_TYPE>();
          assert(pose);
          time_EKF = pose->header.stamp;
        }
        if (terminate) {
          break;
        }

        if (time_GT - start >= startOffset) {
          T_WaBa = sm::kinematics::Transformation(
              Eigen::Vector4d(-pose->transform.rotation.x,
                              -pose->transform.rotation.y,
                              -pose->transform.rotation.z,
                              pose->transform.rotation.w),
              Eigen::Vector3d(pose->transform.translation.x,
                              pose->transform.translation.y,
                              pose->transform.translation.z));

#ifdef VICON
          T_WgBg = sm::kinematics::Transformation(
              Eigen::Vector4d(-trafo->transform.rotation.x,
                              -trafo->transform.rotation.y,
                              -trafo->transform.rotation.z,
                              trafo->transform.rotation.w),
              Eigen::Vector3d(trafo->transform.translation.x,
                              trafo->transform.translation.y,
                              trafo->transform.translation.z)
          );
#endif
#ifdef LEICA
              Eigen::Vector4d(-pose->transform.rotation.x,
                              -pose->transform.rotation.y,
                              -pose->transform.rotation.z,
                              pose->transform.rotation.w),
              Eigen::Vector3d(trafo->point.x,
                              trafo->point.y,
                              trafo->point.z)
        );
#endif

          points1_list.push_back(T_WgBg.t());
          points2_list.push_back(T_WaBa.t());
          if (points1.cols() > 1) {
            ds += (points1.block<3, 1>(0, points1.cols() - 1)
                - points1.block<3, 1>(0, points1.cols() - 2)).norm();
          }
          ds_list.push_back(ds);
        }
      }
      // Find the index to align from.
      int idx_align_start = -1;
      double max_ds = ds_list.at(ds_list.size() - 1);
      for (size_t idx = 0; idx < ds_list.size(); ++idx) {
        if (max_ds - alignment_range <= ds_list.at(idx)) {
          idx_align_start = idx;
          break;
        }
      }
      assert(idx_align_start != -1);
      size_t total_entries = ds_list.size() - idx_align_start;
      points1.conservativeResize(3, total_entries);
      points2.conservativeResize(3, total_entries);
      size_t idx_pt = 0;
      for (size_t idx = idx_align_start; idx < ds_list.size(); ++idx) {
        points1.block<3, 1>(0, idx_pt) = points1_list.at(idx);
        points2.block<3, 1>(0, idx_pt) = points2_list.at(idx);
        ++idx_pt;
      }

      std::cout << "Got " << points1.cols() << "points" << std::endl;
      EstimateRigidTransform(points1, points2, &T_WaWg_range);
      first = false;
    }

    // now find the GT/EKF pairings
    int ctr = 0;  //how many meas did we add this run?
    double ds = 0.0;  // distance travelled
    ros::Time lastTime(0.0);

    MSF_INFO_STREAM(
        "Processing measurements... Current start point: "<<startOffset<<"s into the bag.");

    for (; it_GT != view_GT.end(); ++it_GT) {
      GT_TYPE::ConstPtr trafo = it_GT->instantiate<GT_TYPE>();
      assert(trafo);

      // find closest timestamp
      ros::Time time_GT = trafo->header.stamp + dt;

      EVAL_TYPE::ConstPtr pose = it_EVAL->instantiate<EVAL_TYPE>();
      assert(pose);

      ros::Time time_EKF = pose->header.stamp;

      bool terminate = false;
      //get the measurement close to this GT value
      while (time_GT > time_EKF) {
        it_EVAL++;
        if (it_EVAL == view_EVAL.end()) {
          terminate = true;
          MSF_INFO_STREAM("done. All EKF meas processed!");
          break;
        }
        pose = it_EVAL->instantiate<EVAL_TYPE>();
        assert(pose);
        time_EKF = pose->header.stamp;
      }
      if (terminate) {
        break;
      }

      // add comparison value
      if (time_GT - start >= startOffset) {

        T_WaBa = sm::kinematics::Transformation(
            Eigen::Vector4d(-pose->transform.rotation.x,
                            -pose->transform.rotation.y,
                            -pose->transform.rotation.z,
                            pose->transform.rotation.w),
            Eigen::Vector3d(pose->transform.translation.x,
                            pose->transform.translation.y,
                            pose->transform.translation.z));

#ifdef VICON
        T_WgBg = sm::kinematics::Transformation(
            Eigen::Vector4d(-trafo->transform.rotation.x,
                            -trafo->transform.rotation.y,
                            -trafo->transform.rotation.z,
                            trafo->transform.rotation.w),
            Eigen::Vector3d(trafo->transform.translation.x,
                            trafo->transform.translation.y,
                            trafo->transform.translation.z));
#endif
#ifdef LEICA
        T_WgBg = sm::kinematics::Transformation(
            Eigen::Vector4d(-pose->transform.rotation.x,
                            -pose->transform.rotation.y,
                            -pose->transform.rotation.z,
                            pose->transform.rotation.w),
                    Eigen::Vector3d(trafo->point.x,
                                    trafo->point.y,
                                    trafo->point.z));
#endif

        // initial alignment
        if (ctr == 0) {
//          T_WaWg = (T_WaBa) * T_BaBg * T_WgBg.inverse();
          std::cout << "Transformation single: " << std::endl <<
          ((T_WaBa) * T_BaBg * T_WgBg.inverse()).T() <<std::endl;
          T_WaWg = T_WaWg_range;
          std::cout << "Transformation range: " << std::endl <<
          T_WaWg.T() <<std::endl;
          T_WgBg_last = T_WgBg;
        }

        sm::kinematics::Transformation dT = T_WaBa
            * (T_WaWg * T_WgBg * T_BaBg.inverse()).inverse();
        sm::kinematics::Transformation T_WaBa_gt = (T_WaWg * T_WgBg
            * T_BaBg.inverse());

        T_WaBa_gt = sm::kinematics::Transformation(T_WaBa_gt.q().normalized(),
                                                   T_WaBa_gt.t());
        dT = sm::kinematics::Transformation(dT.q().normalized(), dT.t());

        // update integral
        if (trafo) {
          ds += (T_WgBg.t() - T_WgBg_last.t()).norm();
          // store last GT transformation
          T_WgBg_last = T_WgBg;
        } else {
          // too noisy
          std::cout << " delta|ds| = " << (T_WgBg * T_WgBg_last.inverse()).t().norm() <<std::endl;
          if ((T_WgBg * T_WgBg_last.inverse()).t().norm() > .1) {
            ds += (T_WgBg.t() - T_WgBg_last.t()).norm();
            //MSF_INFO_STREAM((T_WgBg*T_WgBg_last.inverse()).t().norm());
            // store last GT transformation
            T_WgBg_last = T_WgBg;
          }
        }
        Eigen::Vector3d dalpha;
        if (dT.q().head<3>().norm() > 1e-12) {
          dalpha = (asin(dT.q().head<3>().norm()) * 2
              * dT.q().head<3>().normalized());
        } else {
          dalpha = 2 * dT.q().head<3>();
        }
        // g-vector alignment
        Eigen::Vector3d e_z_Wg(0, 0, 1);
        Eigen::Vector3d e_z_Wa = dT.C() * e_z_Wg;
        double dalpha_e_z = acos(std::min(1.0, e_z_Wg.dot(e_z_Wa)));

        if (fabs((time_GT - time_EKF).toSec()) < timeSyncThreshold
            && (time_EKF - lastTime).toSec() > trajectoryTimeDiscretization) {
          if (startOffset.toSec() == startTimeOffset) {
            poses_EVAL << T_WaBa.t().transpose() << ";" << std::endl;
            poses_GT << T_WgBg.t().transpose() << ";" << std::endl;
          }
          outfile << (time_GT - start).toSec() << " " << ds << " "
          << (T_WaBa.t() - T_WaBa_gt.t()).norm()
          << " "  //translation
          << 2 * acos(std::min(1.0, fabs(dT.q()[3]))) << " " << dalpha_e_z
          << " "//orientation
          << T_WaBa.t().transpose() << " "// position estimate
          << T_WgBg.t().transpose() << " "// position ground truth
          << T_WaBa.q().transpose() << " "// orientation estimate
          << T_WgBg.q().transpose() << " "// orientation ground truth
          << std::endl;

          lastTime = time_EKF;  // remember
        }

        // count comparisons
        ctr++;

      }
    }

    MSF_INFO_STREAM("Added "<<ctr<<" measurement edges.");

    //where in the bag should the next eval start
    startOffset += ros::Duration(timeDiscretization);

    if (ctr == 0 || singleRun)  //any new measurements this run?
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

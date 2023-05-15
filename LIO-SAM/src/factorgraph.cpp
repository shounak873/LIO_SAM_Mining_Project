// understand code structure for writing factor graphs
// lets take a look at the two factor graph structures in LIO-SAM


// global optimization graph

// add your headers

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>


using namespace gtsam;

// why do you need this ?
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose


// use std::deque to store your measurements from subscriber

// Double-ended queues are sequence containers with the feature of expansion and contraction on both ends. They are similar to vectors, but are more efficient in case of insertion and deletion of elements. Unlike vectors, contiguous storage allocation may not be guaranteed.
// Double Ended Queues are basically an implementation of the data structure double-ended queue. A queue data structure allows insertion only at the end and deletion from the front. This is like a queue in real life, wherein people are removed from the front and added at the back. Double-ended queues are a special case of queues where insertion and deletion operations are possible at both the ends.
// The functions for deque are same as vector, with an addition of push and pop operations for both front and back.
//
// The time complexities for doing various operations on deques are-
//
// Accessing Elements- O(1)
// Insertion or removal of elements- O(N)
// Insertion or removal of elements at start or end- O(1)
// eg.
// std::deque<nav_msgs::Odometry> gpsQueue;
// gpsQueue.push_back(gps_msg);

// Another very important concept is threads. eg. loop closure is run on a separate thread





// this class will enclose the main structure of the code
class graphOPtimization
{
    // initialize graph and add its components

    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;             //values ?
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    bool firstNode = true;

    // define your subscibers and publishers
    ros::Subscriber subodom; // this odometry can be from lidar or vision
    ros::Subscriber subGPS;

    ros::Publisher pubOdometryGlobal;

    std::deque<nav_msgs::Odometry> gpsQueue; // GPS solutions from some source
    std::deque<nav_msgs::Odometry> odomQueue; // front end estimates of states of each node with respect to world frame

    Eigen::Affine3f currPose;  // pcl::getTranslationAndEulerAngles <=> pcl::getTransformation
    Eigen::Affine3f prevPose;
    Eigen::MatrixXd poseCovariance;

    Eigen::MatrixXd poseCovariance;
    double currentPosetime = 0.0; // this is the time of the current last state node of the robot

    int fkey = 0;
    // constructor
    graphOPtimization()
    {   // isam initialization
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        subGPS   = nh.subscribe<nav_msgs::Odometry> (gpsTopic, 200, &graphOptimization::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        sudOdom = nh.subscribe<nav_msgs::Odometry> (odomTopic, 1, &graohOPtimization::odomHandler, this, ros::TransportHints().tcpNoDelay());

        pubOdometryGlobal = nh.advertise<nav_msgs::Odometry>(globOdomTopic,1);
    }

    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom)
    {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    gtsam::Pose3 odom2Pose3(nav_msgs::Odometry odom)
    {
        float p_x = odom->pose.pose.position.x;
        float p_y = odom->pose.pose.position.y;
        float p_z = odom->pose.pose.position.z;
        float r_x = odom->pose.pose.orientation.x;
        float r_y = odom->pose.pose.orientation.y;
        float r_z = odom->pose.pose.orientation.z;
        float r_w = odom->pose.pose.orientation.w;

        return gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));
    }

    void gpsHandler(const nav_msgs::Odometry::ConstPtr& gpsMsg)
    {
        gpsQueue.push_back(*gpsMsg);
        // std::cout << "GPS QUEUE LENGTH : " << gpsQueue.size() << std::endl;
    }

    void addGPSFactor()
    {
        if (gpsQueue.empty())
        {
            return;
        }
        // wait for system initialized and settles down
        if (fkey < 10)
        {
            return;
        }

        if (poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold)
        {
            return;
        }

        // last gps position
        static Eigen::Vector3f lastGPSPoint;

        while (!gpsQueue.empty())
        {
            if (gpsQueue.front().header.stamp.toSec() < currentPosetime - 0.2)
            {
                // message too old
                gpsQueue.pop_front();
            }
            else if (gpsQueue.front().header.stamp.toSec() > currentPosetime + 0.2)
            {
                // message too new
                break;
            }
            else
            {
                nav_msgs::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                // GPS too noisy, skip
                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];
                if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                    continue;

                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;
                if (!useGpsElevation)
                {
                    gps_z = transformTobeMapped[5];
                    noise_z = 0.01;
                }

                // GPS not properly initialized (0,0,0)
                if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                    continue;

                // Add GPS every a few meters
                Eigen::Vector3f curGPSPoint;
                curGPSPoint[0] = gps_x;
                curGPSPoint[1] = gps_y;
                curGPSPoint[2] = gps_z;
                if ((curGPSPoint - lastGPSPoint).norm() < 5.0)
                    continue;
                else
                    lastGPSPoint = curGPSPoint;

                gtsam::Vector Vector3(3);
                Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
                noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(fkey, gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                gtSAMgraph.add(gps_factor);

                break;
            }
        }

    }

    void odomHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        // odomQueue.push_back(*odomMsg);  Is this needed?

        if (firstNode == true)
        {
            firstPose = odom2Pose3(odomMsg);
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, firstPose, priorNoise));
            initialEstimate.insert(0, firstPose;
            firstNode = false;
            prevPose = odom2affine(odomMsg);
        }
        else{
            // first check that the change of state is large enough. if yes, then add the state as a key
            // otherwise ignore
            Eigen::Affine3f pose = odom2affine(odomMsg);
            Eigen::Affine3f transBetween = prevPose.inverse() * pose;
            if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
                abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
                abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
                sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
            {
                // no new node added to the factor graph
            }
            else{
                noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
                float x, y, z, roll, pitch, yaw;

                pcl::getTranslationAndEulerAngles (prevPose, x, y, z, roll, pitch, yaw);
                gtsam::Pose3 poseprev = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));

                pcl::getTranslationAndEulerAngles (pose, x, y, z, roll, pitch, yaw);
                gtsam::Pose3 posecurr = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));

                gtSAMgraph.add(BetweenFactor<Pose3>(fkey, fkey+1, poseprev.between(posecurr), odometryNoise));
                initialEstimate.insert(key+1, posecurr);

                fkey += 1; // new node added

                prevPose = pose;

                currentPosetime = odomMsg->header.stamp.toSec();
                addGPSFactor();

                isam->update(gtSAMgraph, initialEstimate);
                isam->update();

                // if (aLoopIsClosed == true)  why multiple isam->update() ??
                // {
                //     isam->update();
                //     isam->update();
                //     isam->update();
                //     isam->update();
                //     isam->update();
                // }

                gtSAMgraph.resize(0); // why resizing now ?
                initialEstimate.clear();

                //save key poses
                PointType thisPose3D;
                PointTypePose thisPose6D;
                Pose3 latestEstimate;

                isamCurrentEstimate = isam->calculateEstimate();
                latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
                poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);
                // estimate of last node
                nav_msgs::Odometry laserOdometryROS;
                laserOdometryROS.header.stamp = timeLaserInfoStamp;
                laserOdometryROS.header.frame_id = odometryFrame;
                laserOdometryROS.child_frame_id = "odom_mapping";
                laserOdometryROS.pose.pose.position.x = latestEstimate.translation().x();;
                laserOdometryROS.pose.pose.position.y = latestEstimate.translation().y();;
                laserOdometryROS.pose.pose.position.z = latestEstimate.translation().z();;
                laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(latestEstimate.rotation().roll(), latestEstimate.rotation().pitch(), latestEstimate.rotation().yaw());
                pubOdometryGlobal.publish(laserOdometryROS);
            }

        }

    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    // MO.constTable = constTable;
    std::cout << " Initializing .. " << std::endl;
    graohOPtimization graphOpt();

    ROS_INFO("\033[1;32m----> graph Optimization Started.\033[0m");

    // Threads are important for loop closure or map visualization
    
    // std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
    // std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);

    ros::spin();

    // loopthread.join();
    // visualizeMapThread.join();

    return 0;
}

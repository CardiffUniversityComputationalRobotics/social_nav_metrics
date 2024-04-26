#include "ros/ros.h"
#include <cstdlib>

// octomap
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap/OcTree.h>

// fcl
#include <fcl/fcl.h>
// #include <fcl/collision.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>
#include <fcl/broadphase/broadphase_spatialhash.h>
#include <fcl/common/types.h>
#include <fcl/config.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/math/geometry-inl.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>

#include <nav_msgs/Odometry.h>
#include <pedsim_msgs/AgentStates.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

#include <stdio.h>

// ROS-Octomap interface
using octomap_msgs::GetOctomap;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_counter_node");
    ros::NodeHandle nh("~");

    GetOctomap::Request req;
    GetOctomap::Response resp;

    std::string odom_topic_, agent_states_topic_, collision_counter_topic_, octomap_service_;

    double robot_radius_, robot_height_, agent_radius_;

    nh.param("robot_height", robot_height_, robot_height_);
    nh.param("robot_radius", robot_radius_, robot_radius_);
    nh.param("agent_radius", agent_radius_, agent_radius_);
    nh.param("odom_topic", odom_topic_, odom_topic_);
    nh.param("agent_states_topic", agent_states_topic_, agent_states_topic_);
    nh.param("octomap_service", octomap_service_, octomap_service_);
    nh.param("collision_counter_topic", collision_counter_topic_, collision_counter_topic_);

    std::shared_ptr<fcl::Cylinder<float>> robot_collision_solid_;
    std::shared_ptr<fcl::Cylinder<float>> agent_collision_solid_;

    robot_collision_solid_.reset(new fcl::Cylinder<float>(robot_radius_, robot_height_));
    agent_collision_solid_.reset(new fcl::Cylinder<float>(agent_radius_, 1.5));

    ros::Rate loop_rate(10);

    bool inCollision = false;
    int collisionCounter = 0;
    int collisionCounterAgent = 0;
    int collisionCounterEnviroment = 0;

    ros::Publisher collisionCounterPub = nh.advertise<std_msgs::Int32MultiArray>(collision_counter_topic_, 1000);

    bool foundCollision = false;

    while (ros::ok())
    {

        fcl::CollisionRequestf collision_request;
        fcl::CollisionResultf collision_result_octomap;

        fcl::CollisionResultf collision_result;

        ros::service::call(octomap_service_, req, resp);

        octomap::AbstractOcTree *abs_octree_ = octomap_msgs::msgToMap(resp.map);

        nav_msgs::OdometryConstPtr odomData = ros::topic::waitForMessage<nav_msgs::Odometry>(odom_topic_);

        pedsim_msgs::AgentStatesConstPtr agentStates = ros::topic::waitForMessage<pedsim_msgs::AgentStates>(agent_states_topic_);

        // check if collision with octomap

        if (abs_octree_)
        {

            octomap::OcTree *octree_ = dynamic_cast<octomap::OcTree *>(abs_octree_);

            std::shared_ptr<fcl::OcTreef> tree_ = std::make_shared<fcl::OcTreef>(std::shared_ptr<const octomap::OcTree>(octree_));

            std::shared_ptr<fcl::CollisionObjectf> tree_obj_ = std::make_shared<fcl::CollisionObjectf>((std::shared_ptr<fcl::CollisionGeometry<float>>(tree_)));

            fcl::CollisionObject<float> vehicle_co(robot_collision_solid_);

            vehicle_co.setTranslation(fcl::Vector3f(odomData->pose.pose.position.x, odomData->pose.pose.position.y, robot_height_ / 2.0));

            fcl::collide(tree_obj_.get(), &vehicle_co, collision_request, collision_result_octomap);
        }

        // check if collision with social agents

        foundCollision = false;

        for (int i = 0; i < agentStates->agent_states.size(); i++)
        {

            fcl::CollisionObject<float> vehicle_co(robot_collision_solid_);

            vehicle_co.setTranslation(fcl::Vector3f(odomData->pose.pose.position.x, odomData->pose.pose.position.y, robot_height_ / 2.0));

            pedsim_msgs::AgentState agentState = agentStates->agent_states[i];

            double dRobotAgent =
                std::sqrt(std::pow(agentState.pose.position.x - odomData->pose.pose.position.x, 2) +
                          std::pow(agentState.pose.position.y - odomData->pose.pose.position.y, 2));

            if (dRobotAgent <= (robot_radius_ + agent_radius_))
            {
                // FCL

                fcl::CollisionObject<float> agent_co(agent_collision_solid_);

                agent_co.setTranslation(fcl::Vector3f(agentState.pose.position.x, agentState.pose.position.y, robot_height_ / 2.0));
                fcl::collide(&agent_co, &vehicle_co, collision_request, collision_result);

                if (collision_result.isCollision())
                {
                    foundCollision = true;
                }
            }
        }

        if (!inCollision)
        {
            if (collision_result_octomap.isCollision() || foundCollision)
            {
                collisionCounter++;
                inCollision = true;
                if (collision_result_octomap.isCollision()) {
                    collisionCounterEnviroment++;
                } else {
                    collisionCounterAgent++;
                }
            }
        }
        else if (inCollision)
        {
            if (!collision_result_octomap.isCollision() && !foundCollision)
            {
                inCollision = false;
            }
        }

        std_msgs::Int32MultiArray msg;

        msg.data.clear();
        msg.data.push_back(collisionCounter);
        msg.data.push_back(collisionCounterAgent);
        msg.data.push_back(collisionCounterEnviroment);

        // collisionCounterMsg.data = collisionCounter;

        collisionCounterPub.publish(msg);
        loop_rate.sleep();
    }

    return 0;
}
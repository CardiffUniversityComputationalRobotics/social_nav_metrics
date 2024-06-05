#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pedsim_msgs/msg/agent_states.hpp"
#include "std_msgs/msg/int32.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/srv/get_octomap.hpp"

#include "octomap/octomap.h"
#include "octomap_msgs/conversions.h"
#include "fcl/geometry/octree/octree.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/common/types.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/default_broadphase_callbacks.h"
#include "fcl/broadphase/broadphase_spatialhash.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class CollisionCounterNode : public rclcpp::Node
{
public:
    CollisionCounterNode() : Node("collision_counter_node")
    {
        this->declare_parameter("robot_height", 1.0);
        this->declare_parameter("robot_radius", 0.25);
        this->declare_parameter("agent_radius", 0.3);
        this->declare_parameter("odom_topic", "/odom");
        this->declare_parameter("agent_states_topic", "/pedsim_simulator/simulated_agents");
        this->declare_parameter("octomap_service", "/octomap_full");
        this->declare_parameter("collision_counter_topic", "/collision_counter");

        robot_height_ = this->get_parameter("robot_height").as_double();
        robot_radius_ = this->get_parameter("robot_radius").as_double();
        agent_radius_ = this->get_parameter("agent_radius").as_double();

        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string agent_states_topic = this->get_parameter("agent_states_topic").as_string();
        std::string octomap_service = this->get_parameter("octomap_service").as_string();
        std::string collision_counter_topic = this->get_parameter("collision_counter_topic").as_string();

        robot_collision_solid_ = std::make_shared<fcl::Cylinder<double>>(robot_radius_, robot_height_);
        agent_collision_solid_ = std::make_shared<fcl::Cylinder<double>>(agent_radius_, 1.5);

        collision_counter_ = 0;

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 1, std::bind(&CollisionCounterNode::odom_callback, this, _1));

        agent_states_subscription_ = this->create_subscription<pedsim_msgs::msg::AgentStates>(agent_states_topic, 1, std::bind(&CollisionCounterNode::agent_states_callback, this, _1));

        octomap_client_ = this->create_client<octomap_msgs::srv::GetOctomap>(octomap_service);

        collision_counter_publisher_ = this->create_publisher<std_msgs::msg::Int32>(collision_counter_topic, 1);

        // Check if the service is available
        if (!octomap_client_->wait_for_service(10s))
        {
            RCLCPP_WARN(this->get_logger(), "Octomap service not available");
            return;
        }

        // Create a request and response for the service call
        auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
        auto response = octomap_client_->async_send_request(request);

        // Wait for the response (blocking call)
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), response) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service get_octomap");
            return;
        }

        auto result = response.get()->map;
        octomap::AbstractOcTree *abs_octree = octomap_msgs::msgToMap(result);

        if (!abs_octree)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to convert octomap message to octree");
            return;
        }

        auto octree = dynamic_cast<octomap::OcTree *>(abs_octree);
        auto tree = std::make_shared<fcl::OcTree<double>>(std::make_shared<const octomap::OcTree>(*octree));
        tree_obj_ = std::make_shared<fcl::CollisionObject<double>>(tree);
    }

    void timer_callback();

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_data_ = msg;
    }

    void agent_states_callback(const pedsim_msgs::msg::AgentStates::SharedPtr msg)
    {
        agent_states_ = msg;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<pedsim_msgs::msg::AgentStates>::SharedPtr agent_states_subscription_;
    rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr octomap_client_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr collision_counter_publisher_;

    std::shared_ptr<fcl::Cylinder<double>> robot_collision_solid_, agent_collision_solid_;

    std::shared_ptr<fcl::CollisionObjectd> tree_obj_;

    double robot_height_, robot_radius_, agent_radius_;
    bool in_collision_;
    int collision_counter_ = false;

    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry::SharedPtr odom_data_;
    pedsim_msgs::msg::AgentStates::SharedPtr agent_states_;
};

void CollisionCounterNode::timer_callback()
{

    while (rclcpp::ok())
    {

        if (odom_data_ && agent_states_)
        {

            fcl::CollisionRequest<double> collision_request;
            fcl::CollisionResult<double> collision_result_octomap;
            fcl::CollisionResult<double> collision_result;

            // Check for collision with octomap
            fcl::CollisionObject<double> vehicle_co(robot_collision_solid_);
            vehicle_co.setTranslation(fcl::Vector3<double>(odom_data_->pose.pose.position.x, odom_data_->pose.pose.position.y, robot_height_ / 2.0));
            fcl::collide(tree_obj_.get(), &vehicle_co, collision_request, collision_result_octomap);

            // Check for collision with social agents
            bool found_collision = false;
            for (const auto &agent_state : agent_states_->agent_states)
            {
                double d_robot_agent = std::sqrt(std::pow(agent_state.pose.position.x - odom_data_->pose.pose.position.x, 2) +
                                                 std::pow(agent_state.pose.position.y - odom_data_->pose.pose.position.y, 2));

                if (d_robot_agent <= (robot_radius_ + agent_radius_))
                {
                    fcl::CollisionObject<double> agent_co(agent_collision_solid_);
                    agent_co.setTranslation(fcl::Vector3<double>(agent_state.pose.position.x, agent_state.pose.position.y, robot_height_ / 2.0));
                    fcl::collide(&agent_co, &vehicle_co, collision_request, collision_result);

                    if (collision_result.isCollision())
                    {
                        found_collision = true;
                    }
                }
            }

            if (!in_collision_)
            {
                if (collision_result_octomap.isCollision() || found_collision)
                {
                    collision_counter_++;
                    in_collision_ = true;
                }
            }
            else
            {
                if (!collision_result_octomap.isCollision() && !found_collision)
                {
                    in_collision_ = false;
                }
            }

            std_msgs::msg::Int32 collision_counter_msg;
            collision_counter_msg.data = collision_counter_;
            collision_counter_publisher_->publish(collision_counter_msg);
        }
        rclcpp::spin_some(this->get_node_base_interface());
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    CollisionCounterNode collision_counter_node;
    collision_counter_node.timer_callback();
    rclcpp::shutdown();
    return 0;
}

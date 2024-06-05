#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pedsim_msgs/msg/agent_states.hpp"
#include "std_msgs/msg/int32.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/srv/get_octomap.hpp"
#include "octomap/octomap.h"
#include "fcl/geometry/octree/octree.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/common/types.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/default_broadphase_callbacks.h"
#include "fcl/broadphase/broadphase_spatialhash.h"

using namespace std::chrono_literals;

class CollisionCounterNode : public rclcpp::Node
{
public:
    CollisionCounterNode() : Node("collision_counter_node")
    {
        this->declare_parameter("robot_height", 1.0);
        this->declare_parameter("robot_radius", 0.5);
        this->declare_parameter("agent_radius", 0.3);
        this->declare_parameter("odom_topic", "/odom");
        this->declare_parameter("agent_states_topic", "/agent_states");
        this->declare_parameter("octomap_service", "/get_octomap");
        this->declare_parameter("collision_counter_topic", "/collision_counter");

        double robot_height = this->get_parameter("robot_height").as_double();
        double robot_radius = this->get_parameter("robot_radius").as_double();
        double agent_radius = this->get_parameter("agent_radius").as_double();
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string agent_states_topic = this->get_parameter("agent_states_topic").as_string();
        std::string octomap_service = this->get_parameter("octomap_service").as_string();
        std::string collision_counter_topic = this->get_parameter("collision_counter_topic").as_string();

        robot_collision_solid_ = std::make_shared<fcl::Cylinder<double>>(robot_radius, robot_height);
        agent_collision_solid_ = std::make_shared<fcl::Cylinder<double>>(agent_radius, 1.5);

        timer_ = this->create_wall_timer(
            500ms, std::bind(&CollisionCounterNode::timer_callback, this));

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic,
            1,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                this->odom_callback(msg);
            });

        agent_states_subscription_ = this->create_subscription<pedsim_msgs::msg::AgentStates>(
            agent_states_topic,
            1,
            [this](const pedsim_msgs::msg::AgentStates::SharedPtr msg)
            {
                this->agent_states_callback(msg);
            });

        octomap_client_ = this->create_client<octomap_msgs::srv::GetOctomap>(octomap_service);

        collision_counter_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
            collision_counter_topic,
            1);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_data_ = msg;
    }

    void agent_states_callback(const pedsim_msgs::msg::AgentStates::SharedPtr msg)
    {
        agent_states_ = msg;
    }

    void timer_callback()
    {
        // Check if the service is available
        if (!octomap_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Octomap service not available");
            return;
        }

        // Create a request and response for the service call
        auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
        auto response = octomap_client_->async_send_request(request);

        // Wait for the response (blocking call)
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), response) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service get_octomap");
            return;
        }

        auto result = response.get();
        octomap::AbstractOcTree *abs_octree = octomap_msgs::msgToMap(result->map);

        if (!abs_octree)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to convert octomap message to octree");
            return;
        }

        auto octree = dynamic_cast<octomap::OcTree *>(abs_octree);
        auto tree = std::make_shared<fcl::OcTree<double>>(std::make_shared<const octomap::OcTree>(*octree));
        auto tree_obj = std::make_shared<fcl::CollisionObject<double>>(tree);

        if (!odom_data_ || !agent_states_)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for odometry and agent states data");
            return;
        }

        fcl::CollisionRequest<double> collision_request;
        fcl::CollisionResult<double> collision_result_octomap;
        fcl::CollisionResult<double> collision_result;

        // Check for collision with octomap
        fcl::CollisionObject<double> vehicle_co(robot_collision_solid_);
        vehicle_co.setTranslation(fcl::Vector3<double>(odom_data_->pose.pose.position.x, odom_data_->pose.pose.position.y, robot_height_ / 2.0));
        fcl::collide(tree_obj.get(), &vehicle_co, collision_request, collision_result_octomap);

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

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<pedsim_msgs::msg::AgentStates>::SharedPtr agent_states_subscription_;
    rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr octomap_client_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr collision_counter_publisher_;

    std::shared_ptr<fcl::Cylinder<double>> robot_collision_solid_;
    std::shared_ptr<fcl::Cylinder<double>> agent_collision_solid_;

    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry::SharedPtr odom_data_;
    pedsim_msgs::msg::AgentStates::SharedPtr agent_states_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto collision_counter_node = std::make_shared<CollisionCounterNode>();
    rclcpp::spin(collision_counter_node);
    rclcpp::shutdown();
    return 0;
}

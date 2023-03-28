#include <chrono>
#include <functional>
#include <string>
#include <memory>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#define PI 3.1416

using std::placeholders::_1;
using namespace std::chrono_literals;

const std::string HEAD_PITCH = "head_platform_pitch_joint";
const std::string HEAD_ROLL = "head_platform_roll_joint";
const std::string HEAD_YAW = "head_platform_yaw_joint";

// TODO: Use 'body' move_group once body is ready for testing.
const std::string PLANNING_GROUP = "all";

class FaceFollower : public rclcpp::Node
{
    public:
        FaceFollower()
        : Node("face_follower_node",
                rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        {
//            this->declare_parameter("should_follow", true);
//            this->declare_parameter("foot_z", 0.312);

            foot_z = this->get_parameter("foot_z").as_double();

            should_follow = this->get_parameter("should_follow").as_bool();
            RCLCPP_INFO(
                this->get_logger(),
                "Parameter Set: %s",
                should_follow ? "Following Face" : "Idle");

            // param subscriber to monitor changes to parameters.
            param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

            // Set a callback for changes to this node's "should_follow" parameter.
            auto cb = [this](const rclcpp::Parameter & p) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Parameter Updated: %s",
                    p.as_bool() ? "Following Face" : "Idle");
                should_follow = p.as_bool();
            };
            cb_handle_ = param_sub_->add_parameter_callback("should_follow", cb);

            // Subscribe to /head_in topic
            head_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
                "head_in", 1, std::bind(&FaceFollower::topic_callback, this, _1)
            );
        }

        void setup_moveit(moveit::planning_interface::MoveGroupInterface *move_group)
        {
            move_group_ = move_group;
        }
    private:
        void topic_callback(const geometry_msgs::msg::Point & msg)
        {
            if (!should_follow) {
                return;
            }
            if (msg.x == last_x && msg.y == last_y && msg.z == last_z) {
                return;
            }
            last_x = msg.x;
            last_y = msg.y;
            last_z = msg.z;
            RCLCPP_INFO(this->get_logger(), "Head POS: (%f, %f, %f)",
                    msg.x, msg.y, msg.z);

            // Terrible distance value
            double dist = (1 - msg.z) / foot_z;

            // x- to right; x+ to left
            double delta_x = msg.x - 0.5;
            // y- to down; y+ to up
            double delta_y = 0.5 - msg.y;

            /*
            auto const current_joints = [this]{
                std::map<std::string, double> vals;

                std::vector<std::string> names = move_group_->getJoints();
                std::vector<double> values = move_group_->getCurrentJointValues();

                vals[HEAD_ROLL] = values[find(names.begin(), names.end(), HEAD_ROLL) - names.begin()];
                vals[HEAD_YAW] = values[find(names.begin(), names.end(), HEAD_YAW) - names.begin()];
                vals[HEAD_PITCH] = values[find(names.begin(), names.end(), HEAD_PITCH) - names.begin()];

                return vals;
            }();

            RCLCPP_INFO(this->get_logger(), "Current Platform: (%f, %f, %f)",
                    current_joints.at(HEAD_ROLL), current_joints.at(HEAD_YAW), current_joints.at(HEAD_PITCH));
            */

            // Set target Joint States
            auto const target_joints = [&delta_x, &delta_y, &dist, this]{
                std::map<std::string, double> vals;

                // These values are in radians
//                vals[HEAD_ROLL] = PI / 3.0;
                vals[HEAD_YAW] = asin(delta_x / dist) + last_yaw;
                vals[HEAD_PITCH] = asin(delta_y / dist) + last_pitch;

                return vals;
            }();
            move_group_->setJointValueTarget(target_joints);

            // Create a plan to the target Joint States
            auto const [success, plan] = [this]{
                moveit::planning_interface::MoveGroupInterface::Plan plmsg;
                auto const ok = static_cast<bool>(move_group_->plan(plmsg));
                return std::make_pair(ok, plmsg);
            }();

            // Execute the plan
            if (success) {
                move_group_->execute(plan);
                last_yaw = target_joints.at(HEAD_YAW);
                last_pitch = target_joints.at(HEAD_PITCH);
                RCLCPP_INFO(this->get_logger(), "Successfully Executed!");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Planning Failed!");
            }
        }
        std::shared_ptr<rclcpp::ParameterEventHandler> param_sub_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr head_sub_;
        bool should_follow;
        double foot_z = 1.0;
        moveit::planning_interface::MoveGroupInterface *move_group_;
        double last_x = 0;
        double last_y = 0;
        double last_z = 0;
        // FIXME: Get current state, don't assume start is always 0
        double last_yaw = 0;
        double last_pitch = 0;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FaceFollower>();
    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, PLANNING_GROUP);
    node->setup_moveit(&move_group_interface);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


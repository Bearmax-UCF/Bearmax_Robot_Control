#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "bearmax_msgs/action/task.hpp"

#include "bearmax_moveit/task_server.hpp"

const std::string PLANNING_GROUP = "all";

MoveitTaskServer::MoveitTaskServer()
    : Node("moveit_tasks_node",
            rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true))
{
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Task>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "task",
            std::bind(&MoveitTaskServer::handle_goal, this, _1, _2),
            std::bind(&MoveitTaskServer::handle_cancel, this, _1),
            std::bind(&MoveitTaskServer::handle_accepted, this, _1)
            );

    speech_pub_ = this->create_publisher<std_msgs::msg::String>("speech", 10);

    head_sub_ = this->create_subscription<geometry_msgs::msg::Point>("head_in",
        1, std::bind(&MoveitTaskServer::face_follower, this, _1));

    foot_z = this->get_parameter("foot_z").as_double();

    // Register task executors,
    // task executor functions must be named execute_<task_name>
    REGISTER_TASK(happy);
    REGISTER_TASK(quizzical);
}

/* ========== Task Executors ========== */

// Task: quizzical
void MoveitTaskServer::execute_quizzical(
    const std::shared_ptr<GoalHandleTask> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal: quizzical");

    const JointValueMap target{
        // Cock and uncock head using the action as a toggle.
        {HEAD_ROLL, is_quizzical ? 0.0 : PI / 6.0}
    };

    move_group_->setJointValueTarget(target);

    move_group_->move();

    is_quizzical = !is_quizzical; // Flip is_quizzical bool
    auto result = std::make_shared<Task::Result>();
    result->success = true;
    RESUME_FACE_FOLLOWER(quizzical);
    goal_handle->succeed(result);
}

// Task: happy
void MoveitTaskServer::execute_happy(
    const std::shared_ptr<GoalHandleTask> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal: happy");

    auto const target_list = [this]{
        std::vector<JointValueMap> tlst;

        // Undo quizzical state if needed
        if (is_quizzical) {
            tlst.emplace_back(JointValueMap{
                {HEAD_ROLL, 0.0}
            });
        }

        tlst.emplace_back(JointValueMap{
            {HEAD_ROLL, PI / 6.0}
        });

        tlst.emplace_back(JointValueMap{
            {HEAD_ROLL, 0.0}
        });

        return tlst;
    }();

    int move_cnt = 1;
    for (JointValueMap target : target_list) {
        RCLCPP_INFO(this->get_logger(),
            "Running Move Frame: %d", move_cnt);

        move_group_->setJointValueTarget(target);

        move_group_->move();
        RCLCPP_INFO(this->get_logger(),
            "Finished Move Frame: %d", move_cnt);

        move_cnt++;
    }


    auto result = std::make_shared<Task::Result>();
    result->success = true;
    RESUME_FACE_FOLLOWER(happy);
    goal_handle->succeed(result);
}

/* ==================================== */

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveitTaskServer>();

    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, PLANNING_GROUP);
    node->setup_moveit(&move_group_interface);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <unistd.h>

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

    head_position_error = this->get_parameter("head_position_error").as_double();

    // Register task executors,
    // task executor functions must be named execute_<task_name>
    REGISTER_TASK(happy);
    REGISTER_TASK(quizzical);
    REGISTER_TASK(sad);
    REGISTER_TASK(angry);
    REGISTER_TASK(reset);
    REGISTER_TASK(wave);
    REGISTER_TASK(bump);
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

        auto stateOne = JointValueMap{
            {L_EAR_ROT, (PI / 4.0)},
            {R_EAR_ROT, (PI / -4.0)}
        };

        // Undo quizzical state if needed
        if (is_quizzical) {
            stateOne.insert(std::pair{HEAD_ROLL, 0.0});
        }

        tlst.emplace_back(stateOne);

        auto stateTwo = JointValueMap{
            {L_EAR_ROT, (PI / -4.0)},
            {R_EAR_ROT, (PI / 4.0)}
        };

        tlst.emplace_back(stateTwo);
        // Waggle them ears back and forth!
        tlst.emplace_back(stateOne);
        tlst.emplace_back(stateTwo);
        tlst.emplace_back(stateOne);
        tlst.emplace_back(stateTwo);

        // Now Reset ears back to normal
        tlst.emplace_back(JointValueMap{
            {L_EAR_ROT, 0.0},
            {R_EAR_ROT, 0.0}
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

// Task: sad
void MoveitTaskServer::execute_sad(
    const std::shared_ptr<GoalHandleTask> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal: sad");
    rclcpp::Rate loop_rate(1);

    auto stateOne = JointValueMap{
        {L_EAR_ROT, (PI / 3.0)},
        {R_EAR_ROT, (PI / -3.0)},
        //        {L_EAR_ROT, (PI / 2.0)},
//        {R_EAR_ROT, (PI / 2.0)},
        {HEAD_PITCH, (PI / -4.0)},
        // Crying gesture
        {L_ARM_SHOULDER, (PI / -4.0)},
        {L_ARM_ROTATOR, (PI / 3.0)},
        {L_ARM_ELBOW, (PI)},
        {R_ARM_SHOULDER, (PI / 4.0)},
        {R_ARM_ROTATOR, (PI / -3.0)},
        {R_ARM_ELBOW, (-1 * PI)}
    };

    // Undo quizzical state if needed
    if (is_quizzical) {
        stateOne.insert(std::pair{HEAD_ROLL, 0.0});
    }

    move_group_->setJointValueTarget(stateOne);
    move_group_->move();

    RCLCPP_INFO(this->get_logger(),
            "Staying sad until sadness is cancelled on twitter.");

    auto stateLoopOne = JointValueMap{
        {L_ARM_ROTATOR, (PI / 3.0) + (PI / 6.0)},
        {R_ARM_ROTATOR, (PI / -3.0) - (PI / 6.0)}
    };

    auto stateLoopTwo = JointValueMap{
        {L_ARM_ROTATOR, (PI / 3.0) - (PI / 6.0)},
        {R_ARM_ROTATOR, (PI / -3.0) + (PI / 6.0)}
    };

    bool alt = false;
    // Wait for action to be cancelled
    while (!goal_handle->is_canceling()) {
        move_group_->setJointValueTarget(alt ? stateLoopOne : stateLoopTwo);
        move_group_->move();
        alt = !alt;
        loop_rate.sleep();
    }

    RCLCPP_INFO(this->get_logger(), "Took anti-depressants, no longer sad!");

    // Now Reset everything back to normal
    auto stateLast = JointValueMap{
        {L_EAR_PITCH, 0.0},
        {R_EAR_PITCH, 0.0},
        {L_EAR_ROT, 0.0},
        {R_EAR_ROT, 0.0},
        {HEAD_PITCH, 0.0},
        {L_ARM_SHOULDER, (PI)},
        {L_ARM_ROTATOR, 0.0},
        {L_ARM_ELBOW, 0.0},
        {R_ARM_SHOULDER, (-1 * PI)},
        {R_ARM_ROTATOR, 0.0},
        {R_ARM_ELBOW, 0.0}
    };

    move_group_->setJointValueTarget(stateLast);
    move_group_->move();

    auto result = std::make_shared<Task::Result>();
    result->success = true;
    RESUME_FACE_FOLLOWER(sad);
    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
    } else {
        goal_handle->succeed(result);
    }
}

// Task: angry
void MoveitTaskServer::execute_angry(
    const std::shared_ptr<GoalHandleTask> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal: angry");
    rclcpp::Rate loop_rate(1);

    auto stateOne = JointValueMap{
        {L_EAR_ROT, (PI / -3.0)},
        {R_EAR_ROT, (PI / 3.0)},
//        {L_EAR_ROT, (PI / 2.0)},
//        {R_EAR_ROT, (PI / 2.0)},
        {HEAD_PITCH, (PI / -4.0)}
    };

    // Undo quizzical state if needed
    if (is_quizzical) {
        stateOne.insert(std::pair{HEAD_ROLL, 0.0});
    }

    move_group_->setJointValueTarget(stateOne);
    move_group_->move();

    RCLCPP_INFO(this->get_logger(),
            "Staying angry until cancelled!");

    // Wait for action to be cancelled
    while (!goal_handle->is_canceling()) {
        loop_rate.sleep();
    }

    RCLCPP_INFO(this->get_logger(),
            "Took a chill pill, no longer angry.");

    // Now Reset everything back to normal
    auto stateLast = JointValueMap{
        {L_EAR_PITCH, 0.0},
        {R_EAR_PITCH, 0.0},
        {L_EAR_ROT, 0.0},
        {R_EAR_ROT, 0.0},
        {HEAD_PITCH, 0.0}
    };

    move_group_->setJointValueTarget(stateLast);
    move_group_->move();

    auto result = std::make_shared<Task::Result>();
    result->success = true;
    RESUME_FACE_FOLLOWER(angry);
    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
    } else {
        goal_handle->succeed(result);
    }
}

// Task: reset
void MoveitTaskServer::execute_reset(
    const std::shared_ptr<GoalHandleTask> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal: reset");

    // Now Reset everything back to normal
    auto stateLast = JointValueMap{
        {L_EAR_PITCH, 0.0},
        {R_EAR_PITCH, 0.0},
        {L_EAR_ROT, 0.0},
        {R_EAR_ROT, 0.0},
        {HEAD_PITCH, 0.0},
        {HEAD_ROLL, 0.0},
        {HEAD_YAW, 0.0},
        {R_ARM_SHOULDER, (-1 * PI)},
        {L_ARM_SHOULDER, PI}
    };

    move_group_->setJointValueTarget(stateLast);
    move_group_->move();

    this->last_pitch = 0;
    this->last_yaw = 0;

    auto result = std::make_shared<Task::Result>();
    result->success = true;
    RESUME_FACE_FOLLOWER(reset);
    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
    } else {
        goal_handle->succeed(result);
    }
}


// Task: wave
void MoveitTaskServer::execute_wave(
    const std::shared_ptr<GoalHandleTask> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal: wave");
    rclcpp::Rate loop_rate(1);

    auto const target_list = [this]{
        std::vector<JointValueMap> tlst;

        auto stateOne = JointValueMap{
            {R_ARM_SHOULDER, (PI / 3.0)},
            {L_ARM_SHOULDER, PI},
            {R_ARM_ROTATOR, (PI / -6.0)},
            {R_ARM_ELBOW, (PI / -4.0)},
            {L_EAR_ROT, (PI / 4.0)},
            {R_EAR_ROT, (PI / -4.0)}
        };

        // Undo quizzical state if needed
        if (is_quizzical) {
            stateOne.insert(std::pair{HEAD_ROLL, 0.0});
        }

        tlst.emplace_back(stateOne);

        auto stateTwo = JointValueMap{
            {R_ARM_ELBOW, (PI / 4.0)},
            {L_EAR_ROT, (PI / -4.0)},
            {R_EAR_ROT, (PI / 4.0)}
        };

        tlst.emplace_back(stateTwo);
        // Wave "hand" back and forth
        /*
        tlst.emplace_back(stateOne);
        tlst.emplace_back(stateTwo);
        tlst.emplace_back(stateOne);
        tlst.emplace_back(stateTwo);
        */

        return tlst;
    }();

    int move_cnt = 1;
    while (!goal_handle->is_canceling()) {
        for (JointValueMap target : target_list) {
            RCLCPP_INFO(this->get_logger(),
                "Running Move Frame: %d", move_cnt);

            move_group_->setJointValueTarget(target);

            move_group_->move();
            RCLCPP_INFO(this->get_logger(),
                "Finished Move Frame: %d", move_cnt);

            move_cnt++;
        }
        loop_rate.sleep();
    }

    // Now Reset everything back to normal
    auto stateLast = JointValueMap{
        {R_ARM_SHOULDER, (-1 * PI)},
        {L_ARM_SHOULDER, PI},
        {R_ARM_ROTATOR, 0.0},
        {R_ARM_ELBOW, 0.0},
        {L_EAR_ROT, 0.0},
        {R_EAR_ROT, 0.0}
    };

    move_group_->setJointValueTarget(stateLast);
    move_group_->move();

    auto result = std::make_shared<Task::Result>();
    result->success = true;
    RESUME_FACE_FOLLOWER(wave);
    goal_handle->succeed(result);
}

// Task: bump
void MoveitTaskServer::execute_bump(
    const std::shared_ptr<GoalHandleTask> goal_handle)
{
    rclcpp::Rate loop_rate(1);
    RCLCPP_INFO(this->get_logger(), "Executing goal: bump");

    auto stateOne = JointValueMap{
        {R_ARM_SHOULDER, (PI / -4.0)},
        {R_ARM_ELBOW, (PI / -4.0)}
    };

    // Undo quizzical state if needed
    if (is_quizzical) {
        stateOne.insert(std::pair{HEAD_ROLL, 0.0});
    }

    move_group_->setJointValueTarget(stateOne);
    move_group_->move();

    while (!goal_handle->is_canceling()) {
        loop_rate.sleep();
    }

    auto stateTwo = JointValueMap{
        {R_ARM_SHOULDER, 0.0},
        {R_ARM_ELBOW, 0.0}
    };

    move_group_->setJointValueTarget(stateTwo);
    move_group_->move();

    // Now Reset everything back to normal
    auto stateLast = JointValueMap{
        {R_ARM_SHOULDER, (-1 * PI)},
        {R_ARM_ELBOW, 0.0}
    };

    move_group_->setJointValueTarget(stateLast);
    move_group_->move();

    auto result = std::make_shared<Task::Result>();
    result->success = true;
    RESUME_FACE_FOLLOWER(bump);
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

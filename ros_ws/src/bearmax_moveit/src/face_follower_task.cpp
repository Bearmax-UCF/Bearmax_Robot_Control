#include "bearmax_moveit/task_server.hpp"

void MoveitTaskServer::face_follower(const geometry_msgs::msg::Point & msg)
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
    double dist = (1 - msg.z) / (foot_z * 2.0);

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

    // Move to target
    auto const ok = static_cast<bool>(move_group_->move());
    if (ok) {
        last_yaw = target_joints.at(HEAD_YAW);
        last_pitch = target_joints.at(HEAD_PITCH);
        RCLCPP_INFO(this->get_logger(), "Successfully Executed!");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Execution Failed!");
    }
}

#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <bearmax_behavior/bt_service_node.hpp>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"


using namespace BT;

BT::NodeStatus ShouldFollowFace() {
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FollowFace() {
    printf("Following Face!\n");
    return BT::NodeStatus::SUCCESS;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    BehaviorTreeFactory factory;

    /* Register BT Nodes */

    // Conditions
    factory.registerSimpleCondition("ShouldFollowFace", std::bind(&ShouldFollowFace));

    // Actions
    factory.registerSimpleAction("FollowFace", std::bind(&FollowFace));

    /* ====================== */

//    factory.registerFromROSPlugins();

    auto main_tree_path = ament_index_cpp::get_package_share_directory("bearmax_behavior") + "/trees/main.xml";

    auto tree = factory.createTreeFromFile(main_tree_path);

    // This logger publishes status changes using ZeroMQ. Used by Groot
    PublisherZMQ publisher_zmq(tree, 25);

    NodeStatus status = NodeStatus::IDLE;

    while (rclcpp::ok()) {
        status = tree.tickRoot();
        tree.sleep(std::chrono::milliseconds(100));
    }

    rclcpp::shutdown();
    return 0;
}

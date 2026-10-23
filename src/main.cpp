/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "node.hpp"
#include <iostream>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int code = 0;
    try {
        auto node = std::make_shared<go2::Node>();
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 3);
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception& e) {
        std::cerr << "Go2启动或运行失败：" << e.what() << '\n';
        code = 1;
    }
    rclcpp::shutdown();
    return code;
}

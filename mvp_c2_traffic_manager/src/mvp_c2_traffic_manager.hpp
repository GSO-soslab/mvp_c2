/*
    This file is part of ALPHA AUV project.

    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the project.  If not, see <https://www.gnu.org/licenses/>.

    Author: Jason Miller, jason_miller@uri.edu
    Author: Lin Zhao, linzhao@uri.edu
    Year: 2023-2024

    Copyright (C) 2023-2024 Smart Ocean Systems Laboratory
*/

#pragma once

// c++
#include <thread>

// ros2 standard 
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>


//goby includes
#include <goby/acomms/connect.h>
#include <goby/acomms/amac.h>
#include <goby/acomms/buffer/dynamic_buffer.h>
#include <goby/acomms/queue.h>
#include <goby/acomms/bind.h>
#include <goby/acomms/modem_driver.h>
#include <goby/util/binary.h>
#include <goby/util/debug_logger.h>
#include <goby/util/debug_logger/flex_ostream.h>         // for FlexOs...
#include <goby/util/debug_logger/flex_ostreambuf.h>      // for DEBUG1


class MvpC2TrafficManager : public rclcpp::Node
{

public:
    MvpC2TrafficManager(std::string name = "MvpC2TrafficManager");

    ~MvpC2TrafficManager();

private:

    // ===================================================================== //
    // types
    // ===================================================================== //

    struct Interface
    {
        std::string if_type;
        std::string tcp_address;
        int tcp_port;
        std::string device;
        int baudrate;
    };

    struct MacConfig
    {
        int local_address;
        int max_frame_bytes;
        std::map<int, int> remotes; // remote address, slot time
    };

    struct MessageConfig
    {
        bool ack;
        int blackout_time;
        int max_queue;
        bool newest_first;
        int ttl;
        int value_base;
    };

    struct Config
    {
        MacConfig mac;
        std::map<std::string, MessageConfig> msg;
    };


    // ===================================================================== //
    // global variables
    // ===================================================================== //

    std::thread loop_worker_;

    std::map<std::string, Config> config_;

    goby::acomms::MACManager mac_;

    goby::acomms::DynamicBuffer<std::string> buffer_;

    // ===================================================================== //
    // ROS2 related
    // ===================================================================== //


    // ===================================================================== //
    // functions
    // ===================================================================== //

    void loop();

    /**
     * @brief the goby dccl, mac, queue, and driver are configured and initialized
     *
     */
    void loadConfig();


};
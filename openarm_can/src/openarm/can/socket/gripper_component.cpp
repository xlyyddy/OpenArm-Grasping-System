// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <linux/can.h>
#include <linux/can/raw.h>

#include <iostream>
#include <openarm/can/socket/gripper_component.hpp>

namespace openarm::can::socket {

GripperComponent::GripperComponent(canbus::CANSocket& can_socket)
    : DMDeviceCollection(can_socket) {}

void GripperComponent::init_motor_device(damiao_motor::MotorType motor_type, uint32_t send_can_id,
                                         uint32_t recv_can_id, bool use_fd) {
    // Create the motor
    motor_ = std::make_unique<damiao_motor::Motor>(motor_type, send_can_id, recv_can_id);
    // Create the device with a reference to the motor
    motor_device_ = std::make_shared<damiao_motor::DMCANDevice>(*motor_, CAN_SFF_MASK, use_fd);
    get_device_collection().add_device(motor_device_);
}

void GripperComponent::open(double kp, double kd) { set_position(gripper_open_position_, kp, kd); }

void GripperComponent::close(double kp, double kd) {
    set_position(gripper_closed_position_, kp, kd);
}

void GripperComponent::set_position(double gripper_position, double kp, double kd) {
    if (!motor_device_) return;

    // MIT control to desired position (zero velocity and torque)
    mit_control_one(
        0, damiao_motor::MITParam{kp, kd, gripper_to_motor_position(gripper_position), 0.0, 0.0});
}
}  // namespace openarm::can::socket

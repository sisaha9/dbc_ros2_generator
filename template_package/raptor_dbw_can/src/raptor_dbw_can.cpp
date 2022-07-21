// Copyright (c) 2015-2018, Dataspeed Inc., 2018-2020 New Eagle, All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <cmath>
#include <algorithm>
#include <string>

#include "raptor_dbw_can/raptor_dbw_can.hpp"

using std::chrono::duration;

namespace raptor_dbw_can
{

static constexpr uint64_t MS_IN_SEC = 1000;

RaptorDbwCAN::RaptorDbwCAN(const rclcpp::NodeOptions & options)
: Node("raptor_dbw_can_node", options)
{
  vehicle_number_ = declare_parameter<uint8_t>("vehicle_number", 7);

  dbw_dbc_file_ = declare_parameter<std::string>("dbw_dbc_file", "");
  max_steer_angle_ = declare_parameter<double>("max_steer_angle", 0.0);
  publish_my_laps_ = declare_parameter<bool>("publish_my_laps", true);

  pub_can = this->create_publisher<Frame>(
    "can_rx", 20
  );
  ROS_PUBLISHERS_INITIALIZE

  ROS_SUBSCRIBERS_INITIALIZE
  sub_can_ = this->create_subscription<Frame>(
    "can_tx", 500,
    std::bind(&RaptorDbwCAN::recvCAN, this, std::placeholders::_1)
  );

  dbw_dbc_ = NewEagle::DbcBuilder().NewDbc(dbw_dbc_file_);
}

#define RECV_DBC(handler) \
  message = dbw_dbc_.GetMessageById(id); \
  if (msg->dlc >= message->GetDlc()) {message->SetFrame(msg); handler(msg, message);}

void RaptorDbwCAN::recvCAN(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = nullptr;
  if (!msg->is_rtr && !msg->is_error) {
    auto id = msg->id;
    switch (id) {
      SWITCH_CASE_ID
      default:
        break;
    }
  }
}

RECV_CAN_BODY

RECV_ROS_BODY

}  // namespace raptor_dbw_can


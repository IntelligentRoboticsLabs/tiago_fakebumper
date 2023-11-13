// Copyright 2023 Intelligent Robotics Lab
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

#include <utility>
#include "rclcpp/rclcpp.hpp"

#include "kobuki_ros_interfaces/msg/bumper_event.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class FakeBumper : public rclcpp::Node
{
public:
  FakeBumper()
  : Node("fake_bumper")
  {
    pressed_.state = kobuki_ros_interfaces::msg::BumperEvent::RELEASED;

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "input_scan", rclcpp::SensorDataQoS(),
      std::bind(&FakeBumper::laserCallback, this, _1));

    bumper_pub_ = create_publisher<kobuki_ros_interfaces::msg::BumperEvent>("output_bumper", 10);
  }

  void laserCallback(const sensor_msgs::msg::LaserScan::UniquePtr msg)
  {
    if (pressed_.state) {
      for (size_t i = 0; i < msg->ranges.size(); i++) {
        if (msg->ranges[i] < OBSTACLE_DISTANCE) {
          return;
        }
      }

      pressed_.state = kobuki_ros_interfaces::msg::BumperEvent::RELEASED;
      bumper_pub_->publish(pressed_);
    } else {
      int pos = 0;

      for (size_t i = 0; i < msg->ranges.size(); i++) {
        if (msg->ranges[i] < OBSTACLE_DISTANCE) {
          pressed_.state = kobuki_ros_interfaces::msg::BumperEvent::PRESSED;
          pos = i;
          break;
        }
      }

      if (!pressed_.state) { return; }

      int threshold = msg->ranges.size() / 3;

      if (pressed_.state && pos >= 0 && pos < threshold) {
        pressed_.bumper = kobuki_ros_interfaces::msg::BumperEvent::RIGHT;
      } else if (pressed_.state && pos >= threshold && pos <= threshold * 2) {
        pressed_.bumper = kobuki_ros_interfaces::msg::BumperEvent::CENTER;
      } else if (pressed_.state && pos > threshold * 2 && pos <= msg->ranges.size()) {
        pressed_.bumper = kobuki_ros_interfaces::msg::BumperEvent::LEFT;
      }

      bumper_pub_->publish(pressed_);
    }
  }

private:
  kobuki_ros_interfaces::msg::BumperEvent pressed_;

  static constexpr float OBSTACLE_DISTANCE = 0.1f;

  rclcpp::Publisher<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr bumper_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto fake_bumper_node = std::make_shared<FakeBumper>();
  rclcpp::spin(fake_bumper_node);

  rclcpp::shutdown();

  return 0;
}

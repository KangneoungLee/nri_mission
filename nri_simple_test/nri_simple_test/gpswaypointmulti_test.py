# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from nri_msgs.msg import NriWaypointlistGps


class GpsWaypointMultiPublisher(Node):

    def __init__(self):
        super().__init__('GpsWaypointmulti_test')
        self.publisher_ = self.create_publisher(NriWaypointlistGps, 'multi_gps_cmd_nri', 10)
        timer_period = 3.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = NriWaypointlistGps()
        msg.latitude_goal.append(float(30.6271981))
        msg.longitude_goal.append(float(-96.3303747))
        msg.is_in_field.append(False)
        msg.headang_goal_deg.append(float(60))
        msg.latitude_goal.append(float(30.5242713))
        msg.longitude_goal.append(float(-96.1623354))
        msg.is_in_field.append(True)
        msg.headang_goal_deg.append(float(30))
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    publisher_node = GpsWaypointMultiPublisher()

    rclpy.spin(publisher_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

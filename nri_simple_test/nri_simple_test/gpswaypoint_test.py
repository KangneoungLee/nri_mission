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

from nri_msgs.msg import NriWaypointGps


class GpsWaypointPublisher(Node):

    def __init__(self):
        super().__init__('GpsWaypoint_test')
        self.publisher_ = self.create_publisher(NriWaypointGps, 'gps_cmd_nri', 10)
        timer_period = 3.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = NriWaypointGps()
        msg.latitude_goal = float(30.61736)
        msg.longitude_goal = float(-96.34044)
        msg.is_in_field = False
        msg.headang_goal_deg = float(60)
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    publisher_node = GpsWaypointPublisher()

    rclpy.spin(publisher_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

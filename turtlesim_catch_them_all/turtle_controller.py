#!/usr/bin/env python3

from math import atan2, pi
import math
from urllib import request
import rclpy
from rclpy.node import Node
import re
from turtlesim.msg import Pose
from turtlesim.srv import Kill
from functools import partial
from geometry_msgs.msg import Twist


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.cmd_vel_freq = 10
        self.main_turtle_name = 'main_turtle'
        self.subscribers = []
        self.active_turtles = {}
        self.turtles_info = []

        self.main_turtle_subscriber = self.create_subscription(
            Pose, f'/{self.main_turtle_name}/pose', self.main_turtle_pose_callback, 10)
        self.main_turtle_cmd_vel = self.create_publisher(
            Twist, f'/{self.main_turtle_name}/cmd_vel', 10)
        self.create_timer(1, self.update_positions_of_active_turtles)
        self.create_timer(1/self.cmd_vel_freq, self.get_next_move)
        self.turtle_kill_client = self.create_client(Kill, 'kill')

        self.main_turtle_curr_pose = None
        self.target_pose = None
        self.target_name = ''
        self.dist_tolerance = 0.5
        self.theta_tolerance = 0.1
        self.ka_const = 1
        self.kl_const = 0.5
        self.reached_goal = False

    def update_positions_of_active_turtles(self):
        topics_list = self.get_topic_names_and_types()
        filtered_topics = {topic[0] for topic in topics_list if re.search(
            '/[a-z]+[0-9]+/pose', topic[0]) is not None}

        self.active_turtles = {topic.replace('/', ' ').lstrip().split()[0]: {
            'topic': topic, 'position': Pose()} for topic in filtered_topics}
        for turtle in self.active_turtles:
            available_turtles = [turtle[0] for turtle in self.turtles_info]
            if turtle not in available_turtles:
                self.subscribers.append({turtle: self.create_subscription(
                    Pose, self.active_turtles[turtle]['topic'], partial(self.update_turtle_position, turtle_name=turtle), 1)})
        self.print_active_turtles()

    def update_turtle_position(self, msg, turtle_name):
        available_turtles = [turtle[0] for turtle in self.turtles_info]
        if turtle_name not in available_turtles:
            self.turtles_info.append((turtle_name, msg))
            subscriber_idx = [self.subscribers.index(
                sub) for sub in self.subscribers if turtle_name in sub]
            if len(subscriber_idx) > 0:
                print(
                    f'turtle_name: {turtle_name}, retrieved_name: {self.subscribers[subscriber_idx[0]]}')
                self.destroy_subscription(
                    self.subscribers[subscriber_idx[0]][turtle_name])
                self.subscribers.pop(subscriber_idx[0])

    def get_next_move(self):
        self.get_logger().info(f'Current target: {self.target_name}')
        if len(self.turtles_info) == 0:
            # self.get_logger().warn('No turtles found yet ...')
            return 
        if self.target_pose is None:
            self.set_target()
        dist = self.get_distance_to_target()
        theta_err = self.get_heading_error()
        if dist is None or theta_err is None:
            return
        if dist < self.dist_tolerance:
            self.get_logger().info('Reached goal!')
            self.kill_turtle()
            self.reached_goal = True
            self.target_pose = None
            return None

        msg = Twist()

        if abs(theta_err) > self.theta_tolerance:
            msg.angular.z = self.ka_const * theta_err
        else:
            msg.linear.x = self.kl_const * dist
        # if dist > self.dist_tolerance:
        #     msg.linear.x = self.kl_const * dist
        #     msg.angular.z = self.ka_const * theta_err

        self.main_turtle_cmd_vel.publish(msg)

    def get_heading_error(self):
        if self.target_pose == None:
            return None
        delta_x = self.target_pose.x - self.main_turtle_curr_pose.x
        delta_y = self.target_pose.y - self.main_turtle_curr_pose.y
        target_heading = atan2(delta_y, delta_x)
        heading_error = target_heading - self.main_turtle_curr_pose.theta

        if heading_error > pi:
            heading_error -= 2*pi
        elif heading_error < -pi:
            heading_error += 2*pi
        return heading_error

    def get_distance_to_target(self):
        if self.target_pose == None:
            return None
        curr_pose = [self.main_turtle_curr_pose.x,
                     self.main_turtle_curr_pose.y]
        target_pose = [self.target_pose.x, self.target_pose.y]
        return math.dist(curr_pose, target_pose)

    def kill_turtle(self):
        self.get_logger().info(f'Killing {self.target_name}')
        request = Kill.Request()
        request.name = self.target_name
        future = self.turtle_kill_client.call_async(request)

    def set_target(self):
        if len(self.turtles_info) > 0:
            turtle_name, turtle_pose = self.turtles_info.pop(0)
            self.target_pose = turtle_pose
            self.target_name = turtle_name

    def main_turtle_pose_callback(self, msg):
        self.main_turtle_curr_pose = msg

    def print_active_turtles(self):
        print(self.turtles_info)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

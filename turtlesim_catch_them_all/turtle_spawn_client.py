#!/usr/bin/env python3

from random import randint
from urllib import request, response
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from example_interfaces.msg import String
import re

import random


class TurtleSpawnClientNode(Node):
    def __init__(self):
        super().__init__('turtle_spawn_client')
        self.turtle_name_publisher = self.create_publisher(
            String, 'new_turtle_name', 10)
        self.create_timer(5, self.call_turtle_spawn_service)
        self.spawn_main_turtle()

    def call_turtle_spawn_service(self):
        number_of_active_turtles = self.get_the_number_of_active_turtles()
        if(number_of_active_turtles >= 10):
            self.get_logger().warn(
                f'Exceeded the maximum number of active turtles! (currently {number_of_active_turtles})')
            return
        client = self.create_client(Spawn, 'spawn')
        while not client.wait_for_service(1):
            self.get_logger().warn('Waiting for Spawn service to be up ...')
        request = Spawn.Request()
        request.x = random.random()*10
        request.y = random.random()*10
        request.theta = random.random()

        future = client.call_async(request)
        future.add_done_callback(self.turtle_spawn_future_callback)

    def turtle_spawn_future_callback(self, future):
        try:
            response = future.result()
            self.publish_new_turtle_name(response.name)
        except Exception as e:
            self.get_logger().error(f'Failed to spawn turtle: {e}')

    def spawn_main_turtle(self):
        client = self.create_client(Spawn, 'spawn')
        while not client.wait_for_service(1):
            self.get_logger().warn('Waiting for Spawn service to be up ...')
        request = Spawn.Request()
        request.x = random.random()*10
        request.y = random.random()*10
        request.theta = random.random()
        request.name = 'main_turtle'

        future = client.call_async(request)
        future.add_done_callback(self.turtle_spawn_future_callback)

    def publish_new_turtle_name(self, name):
        msg = String()
        msg.data = name
        self.turtle_name_publisher.publish(msg)

    def get_the_number_of_active_turtles(self):
        topics_list = self.get_topic_names_and_types()
        filtered_topics = {topic[0] for topic in topics_list if re.search(
            '/[a-z]+[0-9]+/pose', topic[0]) is not None}
        for topic in filtered_topics:
            print(topic)
        return len(filtered_topics)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnClientNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

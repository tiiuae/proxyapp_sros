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
# flake8: noqa

from tkinter import S
from fog_msgs.srv import Vec4

import rclpy
from rclpy.node import Node

import socket
from time import sleep  

import threading

import time

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 6862  # Port to listen on (non-privileged ports are > 1023)
import sys
sys.setrecursionlimit(10000)
class UnsecureClient(Node):
    response_from_client = "no_response".encode('utf-8')

    def __init__(self):
        super().__init__('unsecure_service_client')
        self.declare_parameter('host_address', HOST)
        self.declare_parameter('port_number', PORT)
        self.local_ip = self.get_parameter('host_address').value
        self.local_port = self.get_parameter('port_number').value
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        self.s.setsockopt(socket.SOL_TCP, socket.TCP_KEEPIDLE, 1)
        self.s.setsockopt(socket.SOL_TCP, socket.TCP_KEEPINTVL, 1)
        self.s.setsockopt(socket.SOL_TCP, socket.TCP_KEEPCNT, 5)
        self.s.bind((self.local_ip, self.local_port))
        
        self.s.listen(2)
        self.client_socket, adress = self.s.accept()
        self.cli = self.create_client(Vec4, 'gps_waypoint')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Vec4.Request()
        # self.timer = self.create_timer(0.01, self.check_buffer)
        self.check_buffer()

    def close_socket(self):
        self.s.close()
        
    def check_buffer(self):
        self.get_logger().info('Checking buffer...')
        
        if self.client_socket:
            self.get_logger().info('Receive data from buffer...')
            data = self.client_socket.recv(4096)
        
        if data:
            data = data.decode('utf-8')
            self.get_logger().info('I heard: "%s"' % data)
            received_array = eval(data)
            self.get_logger().info('received array: %s' % received_array)
            self.req.goal = received_array
            self.get_logger().info('sending request: %s' % str(self.req))
            future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            self.get_logger().info('Result of gps waypoint: %s %s' % (str(response.success),  str(response.message)))
            response_arr_str = str([response.success, response.message]).encode('utf-8')
            self.get_logger().info('Sending response: %s' % response_arr_str)
            self.client_socket.send(response_arr_str)
        else:
            self.client_socket.close()
            # self.s.listen(2)
            self.client_socket, adress = self.s.accept()
            sleep(1)
        self.check_buffer()
            
def main(args=None):
    rclpy.init(args=args)
    client_node = UnsecureClient()
    rclpy.spin(client_node)
    client_node.close_socket()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
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

from pandas import array
from fog_msgs.srv import Vec4

import rclpy
from rclpy.node import Node

import threading

import time
import socket
from time import sleep

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 6862  # Port to listen on (non-privileged ports are > 1023)

class SecureServer(Node):
    response_from_client = "no_response".encode('utf-8')
    def __init__(self):
        super().__init__('secure_service_server')

        self.declare_parameter('host_address', HOST)
        self.declare_parameter('port_number', PORT)
        self.local_ip = self.get_parameter('host_address').value
        self.local_port = self.get_parameter('port_number').value
        self.unsecure_client_address = (self.local_ip, self.local_port)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # socket for sending cmd
        while self.s.connect_ex(self.unsecure_client_address) != 0:
            self.get_logger().info('Waiting for socket client to connect...')
            sleep(1)
        self.srv = self.create_service(Vec4, 'gps_waypoint', self.service_callback)
        
        self.MAX_TIME_OUT = 2.0
        
    def close_socket(self):
        self.s.close()
        
    def send_command(self, msg):
        command = msg #the actual command string

        self.get_logger().info('I heard: "%s"' % msg)
        try:
            self.s.send(command.encode('utf-8'))
        except:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # socket for sending cmd
            while self.s.connect_ex(self.unsecure_client_address) != 0:
                self.get_logger().info('Waiting for socket client to connect...')
                sleep(1)
            self.s.send(command.encode('utf-8'))
        
        print('sending command: %s to %s' % (command, self.unsecure_client_address))
        start = time.time()
        now = time.time()
        diff = now - start
        if diff > self.MAX_TIME_OUT:
            print('Max timeout exceeded... command %s' % command)
            return
        print('Done!!! sent command: %s to %s' % (command, self.unsecure_client_address))
        
    def service_callback(self, request: Vec4.Request, response: Vec4.Response):
        array_str_utf = str(request.goal.tolist())
        self.get_logger().info('Incoming request\na: %s' % (array_str_utf))
        
        self.send_command(array_str_utf)
        time.sleep(1) #wait for the response
        data = self.s.recv(1024)
        response_parsed = eval(data.decode('utf-8'))# [True, 'succesfully executed']
        response.success = response_parsed[0]
        response.message = response_parsed[1] 
        self.get_logger().info('Request result: %s, message: %s' % (str(response_parsed[0]), response_parsed[1]))
        return response


def main(args=None):
    rclpy.init(args=args)

    server_node = SecureServer()

    rclpy.spin(server_node)
    server_node.close_socket()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
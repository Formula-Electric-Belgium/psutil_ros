#!/usr/bin/env python

#/*******************************************************************************
#* Copyright (c) 2018 Elhay Rauper
#* All rights reserved.
#*
#* Redistribution and use in source and binary forms, with or without
#* modification, are permitted (subject to the limitations in the disclaimer
#* below) provided that the following conditions are met:
#*
#*     * Redistributions of source code must retain the above copyright notice,
#*     this list of conditions and the following disclaimer.
#*
#*     * Redistributions in binary form must reproduce the above copyright
#*     notice, this list of conditions and the following disclaimer in the
#*     documentation and/or other materials provided with the distribution.
#*
#*     * Neither the name of the copyright holder nor the names of its
#*     contributors may be used to endorse or promote products derived from this
#*     software without specific prior written permission.
#*
#* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
#* THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
#* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
#* PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
#* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
#* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
#* IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#* POSSIBILITY OF SUCH DAMAGE.
#*******************************************************************************/

import rclpy
from rclpy.node import Node
import psutil
from psutil_ros.msg import Cpu
from psutil_ros.msg import VirtualMemory
from psutil_ros.msg import Network
from psutil_ros.msg import Temperatures
from psutil_ros.msg import SensorTemp

# parameters
loop_rate = rclpy.parameter.Parameter('~loop_rate', value=1)
publish_cpu = rclpy.parameter.Parameter('~publish_cpu', value=True)
publish_memo = rclpy.parameter.Parameter('~publish_memo', value=True)
publish_temps = rclpy.parameter.Parameter('~publish_temps', value=True)
publish_network = rclpy.parameter.Parameter('~publish_network', value=True)


class PsutilRosNode(Node):
    
    def __init__(self):
        super().__init__('psutil_ros_node')
        self.cpu_pub = self.create_publisher(Cpu, 'psutil_ros/cpu', 10)
        self.mem_pub = self.create_publisher(VirtualMemory, 'psutil_ros/virtual_memory', 10)
        self.temps_pub = self.create_publisher(Temperatures, 'psutil_ros/temps', 10)
        self.network_pub = self.create_publisher(Network, 'psutil_ros/network', 10)

        self.create_timer(1.0, self.timer_callback)


    def timer_callback(self):

        # messages 
        cpu_msg = Cpu()
        memo_msg = VirtualMemory()
        temps_msg = Temperatures()
        network_msg = Network()
	
        #********* CPU *********
        if publish_cpu: 
            cpu_msg.usage = psutil.cpu_percent(interval=0)
            cpu_msg.physical_cores = psutil.cpu_count(logical=False)
            cpu_msg.cores = psutil.cpu_count()
            cpu_msg.current_freq = psutil.cpu_freq().current
            cpu_msg.min_freq = psutil.cpu_freq().min
            cpu_msg.max_freq = psutil.cpu_freq().max

            
            self.cpu_pub.publish(cpu_msg)
        
        #**** Virtual Memo *****
        if publish_memo:
            vm = psutil.virtual_memory()
            memo_msg.total = vm.total
            memo_msg.available = vm.available
            memo_msg.used = vm.used
            memo_msg.free = vm.free
            memo_msg.percent = vm.percent
            
            self.mem_pub.publish(memo_msg)
    
        #***** Temperatures ****  
        if publish_temps:
            temps = psutil.sensors_temperatures()

            for key, value in temps.items():
                sensor_temp_msg = SensorTemp()
                sensor_temp_msg.label = key
                sensor_temp_msg.current = value[0].current if value[0].current is not None else 0.0
                sensor_temp_msg.high = value[0].high if value[0].high is not None else 0.0
                sensor_temp_msg.critical = value[0].critical if value[0].critical is not None else 0.0
                temps_msg.temps.append(sensor_temp_msg)
        
            self.temps_pub.publish(temps_msg)
        
        #******* Network *******   
        if publish_network:
            net = psutil.net_io_counters()
            network_msg.bytes_sent = net.bytes_sent
            network_msg.bytes_recv = net.bytes_recv
            network_msg.packets_sent = net.packets_sent
            network_msg.packets_recv = net.packets_recv
            network_msg.errin = net.errin
            network_msg.errout = net.errout
            network_msg.dropin = net.dropin
            network_msg.dropout = net.dropout
            
            self.network_pub.publish(network_msg)


def main(args=None):
    rclpy.init(args=args)
	
    psutil_ros_node = PsutilRosNode()

    rclpy.spin(psutil_ros_node)

    psutil_ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

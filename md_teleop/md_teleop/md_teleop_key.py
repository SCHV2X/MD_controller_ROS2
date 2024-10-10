#!/usr/bin/env python3
import rclpy
import sys, signal, os
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from getkey import getkey

MAX_VEL = 1000
VEL_STEP_SIZE = 5

msg = """
-------------------------------------------------
CCW : [ CW : ]   

CTRL-C to quit
-------------------------------------------------
"""

class TeleopKey(Node):
    def __init__(self):
        super().__init__("md_teleop_key_node")

        qos_profile = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)

        self.cmd_rpm_pub = self.create_publisher(Int32MultiArray, "/cmd_rpm", qos_profile)
        signal.signal(signal.SIGINT, self.signal_handler) #callback if ctrl+C signal is input.

        target_rpm1 = 0
        target_rpm2 = 0
        os.system('clear')
        print(msg)

        while(1):
            key = getkey()
            if key == "w":
                target_rpm1 = self.CheckLRPMLimit(target_rpm1 + VEL_STEP_SIZE)
                target_rpm2 = self.CheckLRPMLimit(target_rpm2 + VEL_STEP_SIZE)
                os.system('clear')
                print(msg)
                print("target_RPM = ", target_rpm1, ", ", target_rpm2)
            elif key == "s":
                target_rpm1 = self.CheckLRPMLimit(target_rpm1 - VEL_STEP_SIZE)
                target_rpm2 = self.CheckLRPMLimit(target_rpm2 - VEL_STEP_SIZE)
                os.system('clear')
                print(msg)
                print("target_RPM = ", target_rpm1, ", ", target_rpm2)
            elif key == "a":
                target_rpm1 = self.CheckLRPMLimit(target_rpm1 - VEL_STEP_SIZE)
                target_rpm2 = self.CheckLRPMLimit(target_rpm2 + VEL_STEP_SIZE)
                os.system('clear')
                print(msg)
                print("target_RPM = ", target_rpm1, ", ", target_rpm2)
            elif key == "d":
                target_rpm1 = self.CheckLRPMLimit(target_rpm1 + VEL_STEP_SIZE)
                target_rpm2 = self.CheckLRPMLimit(target_rpm2 - VEL_STEP_SIZE)
                os.system('clear')
                print(msg)
                print("target_RPM = ", target_rpm1, ", ", target_rpm2)
            
            rpm = Int32MultiArray()
            rpm.data = [target_rpm1, target_rpm2]
            self.cmd_rpm_pub.publish(rpm)
    

    def signal_handler(self, sig, frame):
        rpm = Int32MultiArray()
        rpm.data = [0, 0]
        print("target_RPM = ", rpm.data)

        self.cmd_rpm_pub.publish(rpm)
            
        sys.exit(0)

    def CheckLRPMLimit(self, vel):
        if vel <= -MAX_VEL:
            vel = -MAX_VEL
        elif vel >= MAX_VEL:
            vel = MAX_VEL
        
        return vel

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKey()
    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
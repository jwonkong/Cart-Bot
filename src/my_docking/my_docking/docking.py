from my_action_interface.action import DockingTwo
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
import numpy as np
from scipy.spatial.transform import Rotation
import time
from collections import deque

class DockingTwoActionServer(Node):
    def __init__(self):
        
        # super().__init__('dockingtwo_action_server')
        # # action server 생성
        # self.action_server = ActionServer(
        #     self,
        #     DockingTwo,
        #     'dockingtwo',
        #     self.execute_callback,
        #     goal_callback = self.goal_callback,
        # )

        self.subscription = self.create_subscription(
                PoseArray,
                '/aruco_poses',
                self.pose_callback,
                10
            )
        
        self.get_logger().info('### DockingTwo Action Server Started')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # PID init
        self.prev_error = 0.0
        self.integral = 0.0
        self.kp = 2
        self.ki = 0.6
        self.kd = 80
        
        
        # Save previous value
        # Save previous values using deque
        self.prev_cmd_angular_z = deque(maxlen=10)
        self.prev_angle_degrees = deque(maxlen=10)
        
        
        # Angular velocity limits
        self.max_angular_z = 3.0
        self.min_angular_z = 0.1
        
        # count
        self.cnt_num = 0
        self.cnt_near = 0
        self.cnt_time = 0
        self.first = True
        self.ok = False
        self.again = False
        self.cnt1, self.cnt2, self.ccnt = 0, 0, 0

    def pose_callback(self, msg):
        # self.get_logger().info('Executing goal')
        if len(msg.poses) > 0 and self.start:

            # 포즈 정보를 개별 변수에 저장
            pose_x = msg.poses[0].position.x
            # pose_y = msg.poses[0].position.y
            pose_z = msg.poses[0].position.z
            pose_qx = msg.poses[0].orientation.x
            pose_qy = msg.poses[0].orientation.y
            pose_qz = msg.poses[0].orientation.z
            pose_qw = msg.poses[0].orientation.w

            object_orientation = np.array([pose_qx, pose_qy, pose_qz, pose_qw])
            rotation_matrix = Rotation.from_quat(object_orientation).as_matrix()
            object_direction = rotation_matrix[:, 0]
            my_direction = np.array([1, 0, 0])
            angle_radians = np.arccos(np.dot(object_direction, my_direction))
            angle_degrees = np.degrees(angle_radians)
            self.get_logger().info('angle: %f' % angle_degrees)                   
            
            error = -0.03 - pose_x
            self.integral += error
            derivative = error - self.prev_error
            
            angular_z = self.kp * error + self.ki * self.integral + self.kd * derivative
                        
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = -0.2
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.linear.z = 0.0
            cmd_vel_msg.angular.x = 0.0
            cmd_vel_msg.angular.y = 0.0
            cmd_vel_msg.angular.z = angular_z
            
            
            # 일정거리 가까워짐
            if(pose_z < 0.7 and self.ok == False and self.again == False):
                self.get_logger().info('##################################poze_z < 0.7##################################')
                
                # 모든게 완벽, 결합!
                if(pose_z < 0.7 and error < 0.04 and angle_degrees < 10.0):
                    self.cnt_near += 1 # 튀는 값 잡

                    if(angle_degrees < 10.0 and self.cnt_near >= 5):
                        self.get_logger().info('////////////////////Wait////////////////////')
                        time.sleep(2)
                        
                        cmd_vel_msg.linear.x = -0.3
                        cmd_vel_msg.angular.z = 0.0                    
                        self.get_logger().info('&&&&&&&&&&Docking&&&&&&&&&&')
                        
                        while(self.cnt_time < 200000):
                            self.cnt_time += 1
                            self.publisher.publish(cmd_vel_msg)
                    
                        cmd_vel_msg.linear.x = 0.0    
                        self.publisher.publish(cmd_vel_msg)
                        self.ok = True
                        
                # 제자리 돌기가 필요한 상황
                elif(pose_z < 0.7 and (error >= 0.04 and angle_degrees >= 10.0) and self.ccnt < 30):
                    cmd_vel_msg.linear.x = 0.0
                    cmd_vel_msg.angular.z = angular_z
                    self.ccnt += 1
                    self.get_logger().info('Turn1')
                
                # 후진 후 재진입
                else:
                    self.ccnt = 0
                    self.prev_error = 0.0
                    self.integral = 0.0
                    self.again = True
                
            # 다시 시도하기 위한 후진
            if(self.again == True):
                self.get_logger().info('@@@@@@@@@@Again@@@@@@@@@@')
                if(pose_z < 0.7):
                    cmd_vel_msg.linear.x = 0.2
                    cmd_vel_msg.linear.x = 0.1
                    
                elif(angle_degrees >= 4.0 and self.prev_cmd_angular_z[-1] >= 0.1):
                    if (angle_degrees - (sum(self.prev_angle_degrees) / len(self.prev_angle_degrees)) <= 0): # Good
                        self.cnt1 += 1
                        if(self.cnt1 > 10):
                            cmd_vel_msg.angular.z = (sum(self.prev_angle_degrees)/abs(sum(self.prev_angle_degrees)))*0.1
                            self.cnt1 = 0
                            self.cnt2 = 0
                            
                    else: # Bad
                        self.cnt2 += 1      
                        if(self.cnt2 > 10):
                            self.cnt1 = 0
                            self.cnt2 = 0
                            cmd_vel_msg.angular.z = -(sum(self.prev_angle_degrees)/abs(sum(self.prev_angle_degrees)))*0.1
                            
                    cmd_vel_msg.linear.x = 0.0
                    self.get_logger().info('Turn2')
                
                elif((pose_z <= 1.2 and error < 0.04 and angle_degrees < 10.0) or pose_z > 1.2):
                    self.again = False  
                    self.first = True
                    self.prev_error = 0.0
                    self.integral = 0.0
                    self.cnt_near = 0
                    self.get_logger().info('##########GOGO##########')
                
                else:
                    cmd_vel_msg.linear.x = 0.2
                    cmd_vel_msg.angular.z = angular_z
                    
            # 도킹 끝        
            if(self.ok == True and pose_z < 0.7):
                time.sleep(2)
                cmd_vel_msg.linear.x = 1.0
                cmd_vel_msg.angular.z = 0.0
                while(self.cnt_num < 50000):
                    self.cnt_num += 1
                    self.publisher.publish(cmd_vel_msg)
                rclpy.shutdown()
                return 0
                
            
            # 제어값을 제한하여 너무 큰 값, 작은 값이 발생하지 않도록 함
            if cmd_vel_msg.angular.z > self.max_angular_z:
                cmd_vel_msg.angular.z = self.max_angular_z
            elif cmd_vel_msg.angular.z < -self.max_angular_z:
                cmd_vel_msg.angular.z = -self.max_angular_z
                  
            if abs(cmd_vel_msg.angular.z) < self.min_angular_z:
                if (cmd_vel_msg.angular.z > 0):
                    cmd_vel_msg.angular.z = self.min_angular_z
                elif (cmd_vel_msg.angular.z < 0):
                    cmd_vel_msg.angular.z = -self.min_angular_z
                
            self.publisher.publish(cmd_vel_msg)
            
            # self.get_logger().info('Cmd linear x: %f' % cmd_vel_msg.linear.x)
            self.get_logger().info('Cmd angular z: %f' % cmd_vel_msg.angular.z)
            self.get_logger().info('Distance: %f' % pose_z)
            self.get_logger().info('Error: %f' % error)
            self.get_logger().info(' ')
            
            # Save prev
            self.prev_error = error
            # self.prev_cmd_angular_z = cmd_vel_msg.angular.z
            # self.prev_angle_degrees = angle_degrees
            self.prev_cmd_angular_z.append(cmd_vel_msg.angular.z)
            self.prev_angle_degrees.append(angle_degrees)
            
            ###############
            
        # # goal_handle
        # goal_handle.succeed()
        # self.get_logger().warn('### Succeed ###')

        # # result response
        # result = DockingTwo.Result()
        # # result.sequence = feedback_msg.partial_sequence
            
        # return result
    
    # def execute_callback(self, goal_handle):
    #     self.start = True
        

    # # goal request 가 sever 도달 시 처음 진입하게 되는 callback
    # def goal_callback(self, goal_request):
    #     "Accept or reject a client request to begin an action."
    #     self.get_logger().info('Received goal request')
        
    #     return GoalResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)
    node = DockingTwoActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

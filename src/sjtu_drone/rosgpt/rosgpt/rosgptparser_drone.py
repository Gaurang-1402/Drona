import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import copy
import math
import rclpy 
from math import atan2
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import logging

# Configure the logger
logging.basicConfig(level=logging.ERROR)

# Now, twist_msg.linear.x is x, twist_msg.linear.y is y, twist_msg.linear.z is z
from geometry_msgs.msg import Pose
import time
from rclpy.executors import SingleThreadedExecutor
import threading
from concurrent.futures import ThreadPoolExecutor
from functools import partial
from std_msgs.msg import Empty

def quat2Yaw(qw, qx, qy, qz):
    '''
    Translates from Quaternion to Yaw. 

    @param qw,qx,qy,qz: Quaternion values

    @type qw,qx,qy,qz: float

    @return Yaw value translated from Quaternion

    '''
    rotateZa0=2.0*(qx*qy + qw*qz)
    rotateZa1=qw*qw + qx*qx - qy*qy - qz*qz
    rotateZ=0.0
    if(rotateZa0 != 0.0 and rotateZa1 != 0.0):
        rotateZ=atan2(rotateZa0,rotateZa1)
    return rotateZ

class DroneController(Node):

    def __init__(self):
        super().__init__('drone_controller')
        self.create_subscription(String,'/voice_cmd',self.voice_cmd_callback,10)
        self.velocity_publisher = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, "/drone/gt_pose", self.pose_callback, 10)
        self.x = 0.0
        self.y  = 0.0
        self.theta  = 0.0
        self.pose = Pose()
        
        self.takeoff_publisher = self.create_publisher(Empty, '/drone/takeoff', 10)
        self.land_publisher = self.create_publisher(Empty, '/drone/land', 10)

        # self.create_timer(2.0, self.publish_velocity)
        
        self.thread_executor = ThreadPoolExecutor(max_workers=1)

        self.move_executor = SingleThreadedExecutor()
        move_thread = threading.Thread(target=self.move_executor.spin)
        move_thread.start()
        print('ROSGPT Drone Controller Started. Waiting for input commands ...')
    
    def pose_callback(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        self.z = msg.position.z
        self.theta = msg.orientation
        self.pose = msg

    def takeoff(self):
        takeoff_msg = Empty()
        self.takeoff_publisher.publish(takeoff_msg)

    def land(self):
        land_msg = Empty()
        self.land_publisher.publish(land_msg)

    def stop(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)


    #this callback represents the ROSGPTParser. It takes a JSON, parses it, and converts it to a ROS 2 command
    def voice_cmd_callback(self, msg):
        #print(msg.data)
        try:

            print(msg.data)

            commands = json.loads(msg.data.strip("'").replace("'", '"'))

            print(commands)
            
            for cmd in commands:
                cmd = cmd['command']
                
                if cmd["action"] == 'takeoff':
                    print("Takeoff functionality")
                    self.thread_executor.submit(self.takeoff)

                elif cmd["action"] == 'land':
                    self.thread_executor.submit(self.land)
                    
                elif cmd["action"] == 'stop':
                    self.thread_executor.submit(self.stop)

                elif cmd["action"] == 'move':
                    linear_speed = cmd["params"].get('linear_speed', 0.2)
                    distance = cmd["params"].get('distance', 1.0)
                    direction = cmd["params"].get('direction', "forward")

                    print(f'linear_speed: {linear_speed}, distance: {distance}, direction: {direction}')
                    
                    # METHOD: Create a thread executor
                    # we need to run the method on a different thread to avoid blocking rclpy.spin. 
                    self.thread_executor.submit(self.move, linear_speed, distance, direction)

                    # running move on the main thread will generate to error, as it will block rclpy.spin
                    # self.move(linear_speed, distance, direction)

                ## TODO: langchain sync up
                # elif cmd['command'] == 'rotate':
                #     angular_velocity = cmd.get('angular_velocity', 1.0)
                #     angle = cmd.get('angle', 90.0)
                #     is_clockwise = bool(cmd.get('is_clockwise', True))
                #     self.thread_executor.submit(self.rotate, angular_velocity, angle, is_clockwise)
                    # self.rotate(angular_velocity, angle, is_clockwise)

        except json.JSONDecodeError as e:
            logging.exception('[json.JSONDecodeError] Invalid or empty JSON string received: %s', msg.data)
        except Exception as e:
            logging.exception('[Exception] An unexpected error occurred: %s', str(e)) 


    def get_distance(self, start, destination):
        return math.sqrt(
            ((destination.position.x - start.position.x) ** 2) +
            ((destination.position.y - start.position.y) ** 2) +
            ((destination.position.z - start.position.z) ** 2)
        )


    def move(self, linear_speed, distance, direction): 
        print(f'Start moving the drone {direction} at {linear_speed} m/s for a distance of {distance} meters')

        if abs(linear_speed) > 1.0:
            print('[ERROR]: The speed in any direction must be lower than 1.0!')
            return -1
        
        
        linear_vector = Vector3()

        try: 
            if direction == "forward":
                linear_vector.x = linear_speed
                linear_vector.y = 0.0
                linear_vector.z = 0.0

            elif direction == "backward":
                linear_vector.x = -linear_speed
                linear_vector.y = 0.0
                linear_vector.z = 0.0
            elif direction == "left":
                linear_vector.x = 0.0
                linear_vector.y = linear_speed
                linear_vector.z = 0.0
            elif direction == "right":
                linear_vector.x = 0.0
                linear_vector.y = -linear_speed
                linear_vector.z = 0.0
            elif direction == "up":
                linear_vector.x = 0.0
                linear_vector.y = 0.0
                linear_vector.z = linear_speed
            elif direction == "down":
                linear_vector.x = 0.0
                linear_vector.y = 0.0
                linear_vector.z = -linear_speed

        except Exception as e:
            print('[Exception] An unexpected error occurred:', str(e))

        twist_msg = Twist()
        twist_msg.linear = linear_vector

        try:
            start_pose = copy.copy(self.pose)

            print('start_pose: ', start_pose)
            print('current_pose: ', self.pose)
            while self.get_distance(start_pose, self.pose) < distance:

                print('distance moved: ', self.get_distance(start_pose, self.pose))

                self.velocity_publisher.publish(twist_msg)
                self.move_executor.spin_once(timeout_sec=0.5)
        except Exception as e:

            print('[Exception] An unexpected error occurred:', str(e))

        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0

        print("Stopping the drone ...")

        self.velocity_publisher.publish(twist_msg)

        # print('distance moved: ', self.get_distance(start_pose, self.pose))
        print('The Robot has stopped...')


    def rotate (self, angular_speed_degree, desired_relative_angle_degree, clockwise):
        print('Start Rotating the Robot ...')
        #rclpy.spin_once(self)

        print("Stopping the drone ...")
        self.stop()

        twist_msg=Twist()
        angular_speed_degree=abs(angular_speed_degree) #make sure it is a positive relative angle
        if (angular_speed_degree>30) :
            print (angular_speed_degree)
            print('[ERROR]: The rotation speed must be lower than 0.5!')
            return -1
        
        angular_speed_radians = math.radians(angular_speed_degree)
        twist_msg.angular.z = -abs(angular_speed_radians) if clockwise else abs(angular_speed_radians)
        # twist_msg.angular.z = abs(angular_speed_radians) * (-1 if clockwise else 1)

        start_pose = copy.copy(self.pose)
        
        #rclpy.spin_once(self)

        rotated_related_angle_degree=0.0

        while rotated_related_angle_degree<desired_relative_angle_degree:
            self.velocity_publisher.publish(twist_msg)
            start_theta = quat2Yaw(start_pose.orientation.w, start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z)
            pose_theta = quat2Yaw(self.pose.orientation.w, self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z)
            rotated_related_angle_degree = math.degrees(abs(start_theta - pose_theta))
            print('angle rotated: ', rotated_related_angle_degree)
            time.sleep(0.01)

        print ('rotated_related_angle_degree', rotated_related_angle_degree, 'desired_relative_angle_degree', desired_relative_angle_degree)

        self.stop()
        print('The Robot has stopped...')

        return 0

    
def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

"""ROS2 IMPORTS"""
import rclpy
from rclpy.node import Node
from vicon_receiver.msg import Position # Ensure you haave the corresponding package for this custom message type
from vicon_receiver.msg import PositionList # Ensure you haave the corresponding package for this custom message type
from turtlesim.msg import Pose

"""ROBOT ACTUATOR IMPORTS"""
import RPi.GPIO as GPIO
import Adafruit_PCA9685

"""GENERAL PURPOSE PYTHON IMPORTS"""
import numpy as np
import math
import time 
import json
#======================define motor control=========================
# This code is for controlling a robot car using a Raspberry Pi and Adafruit PCA9685
# It uses GPIO pins to control the direction of the motors and the PCA9685 to control the speed of the motors.
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


IN1 = 23
IN2 = 24
IN3 = 27
IN4 = 22

ENA = 0
ENB = 1

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

def changespeed_left(speed):
    pwm.set_pwm(ENA, 0, speed)

def changespeed_right(speed):
    pwm.set_pwm(ENB, 0, speed)

def stopcar():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm.set_pwm(ENA, 0, 0)
    pwm.set_pwm(ENB, 0, 0)

def custom_speed(speed_left, speed_right):
    #make all motors moving forward at the speed of variable move_speed
    if speed_left < 0:
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN1, GPIO.HIGH)
        speed_left = -speed_left
    else:
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN1, GPIO.LOW)

    if speed_right < 0:
        GPIO.output(IN4, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        speed_right = -speed_right
    else:
        GPIO.output(IN4, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)

    speed_left += 500
    speed_right += 500

    changespeed_left(speed_left)
    changespeed_right(speed_right)

def calculate_speed(v_lin, v_ang):
    """Calculate the speed for left and right motors based on linear and angular velocities."""
    right_motor = 11.81 * v_ang + 8.12307 * v_lin - 182.898 
    left_motor = -11.81 * v_ang + 8.12307 * v_lin - 164.878

    return left_motor, right_motor

#======================define helper functions=========================
def angle_diff(a1, a2):
    if isinstance(a1, np.ndarray):
        diff = a1 - a2
        diff[diff > 180] = diff[diff > 180] - 360
        diff[diff < -180] = diff[diff < -180] + 360
    else:
        diff = a1 - a2
        if diff > 180:
            return diff - 360
        if diff < -180:
            return 360 + diff

    return diff

def quaternion_to_yaw(w, x_rot, y_rot, z_rot):
    # Compute yaw (heading) angle in radians
    yaw = math.atan2(2.0 * (w * z_rot + x_rot * y_rot), 1.0 - 2.0 * (y_rot**2 + z_rot**2))
    return yaw
#======================define parameters=============================
with open("parameters.json", 'r') as f:
    parameters = json.load(f)

class Options():
    def __init__(self):
        self.corner1 = (float(parameters["corner1"][0]), float(parameters["corner1"][1]))  # limit moving area x, y
        self.corner2 = (float(parameters["corner2"][0]), float(parameters["corner2"][1]))
        self.rot_index = int(parameters["rot_index"])  # message index of rotation angle
        self.update_time = float(parameters["update_time"])  # time for updating current position in seconds
        self.minimal_dist = float(parameters["minimal_dist"])  # distance defining two car too close
        self.away_scale = float(parameters["away_scale"])  # amount of going away when two cars are too close
        self.angular_speed = float(parameters["angular_speed"])
        self.rotate_scale = float(parameters['rotate_scale'])

opt = Options()

#======================define global pose variable=========================
pos_message = {}

#======================define vicon receiver node=========================

class ViconSubscriber(Node):
    """Constants"""

    def __init__(self, move_period=3):
        super().__init__('vicon_subscriber')
        self.x_low = min(opt.corner1[0], opt.corner2[0])
        self.x_high = max(opt.corner1[0], opt.corner2[0])
        self.y_low = min(opt.corner1[1], opt.corner2[1])
        self.y_high = max(opt.corner1[1], opt.corner2[1])

        self.center = ((self.x_low + self.x_high) / 2., (self.y_low + self.y_high) / 2.)
        self.x_span = abs(self.x_high - self.x_low)
        self.y_span = abs(self.y_high - self.y_low)

        # Identification name of this subject as given on vicon tracking system
        self.id = ""
        """Stores the current position of the robot. As specified in the vicon_receiver.msg.Position message:
        x_trans: x translation in meters
        y_trans: y translation in meters
        z_trans: z translation in meters
        x_rot: x rotation in radians
        y_rot: y rotation in radians
        z_rot: z rotation in radians
        w: w rotation in radians
        string subject_name: name of the subject
        string segment_name: name of the segment
        int frame_number: unit of time for each capture
        string translation_type: local or global"""
        self.current_position = Position()
        self.subscription = self.create_subscription(
            PositionList,
            "vicon/default/data",
            self.listener_callback,
            10)
        
        self.timer = self.create_timer(move_period, self.move_callback)

    def move_callback(self):
        # random sample move destination
        next_x = np.random.rand(1)
        next_y = np.random.rand(1)
        # next_x = self.map_position(next_x)
        # next_y = self.map_position(next_y)
        next_x, next_y = self.map_position(next_x, next_y)

        self.move_to(next_x, next_y)

    def listener_callback(self, msg):
        for i in range(msg.n):
            if msg.positions[i].subject_name == self.id:
                # Update the current position with the received data
                self.current_position = msg.positions[i]
                pos_message['self'] = (float(msg.positions[i].x_trans), float(msg.positions[i].y_trans), float(quaternion_to_yaw(
                    msg.positions[i].w, 
                    msg.positions[i].x_rot, 
                    msg.positions[i].y_rot, 
                    msg.positions[i].z_rot)))
            else:
                pos_message[msg.positions[i].subject_name] = (float(msg.positions[i].x_trans), float(msg.positions[i].y_trans), float(quaternion_to_yaw(
                    msg.positions[i].w, 
                    msg.positions[i].x_rot, 
                    msg.positions[i].y_rot, 
                    msg.positions[i].z_rot)))
                
            self.get_logger().info('subject "%s" with segment %s:' %(msg.positions[i].subject_name, msg.positions[i].segment_name))
            self.get_logger().info('I heard translation in x, y, z: "%f", "%f", "%f"' % (msg.positions[i].x_trans, msg.positions[i].y_trans, msg.positions[i].z_trans))
            self.get_logger().info('I heard rotation in x, y, z, w: "%f", "%f", "%f", "%f": ' % (msg.positions[i].x_rot, msg.positions[i].y_rot, msg.positions[i].z_rot, msg.positions[i].w))

    
    #======================define move_to function=========================
    def move_to(self, next_x, next_y, sleep_time=None):
        global pos_message

        goal_message = Pose()
        goal_message.x = float(next_x)
        goal_message.y = float(next_y)


        if len(pos_message.keys()) == 0:
            print("position not received")
            return
        else:
            target_vec = (next_x - pos_message['self'][0], next_y - pos_message['self'][1])

            target_dist = math.sqrt( (next_x - pos_message['self'][0]) ** 2 \
                                    + (next_y - pos_message['self'][1]) ** 2 )

            # check if it's angular or radius
            target_angle = np.sign(target_vec[1]) * \
                            math.acos(target_vec[0] / target_dist)
            target_angle = target_angle * 180 / np.pi

            angle_to_move = -angle_diff(pos_message['self'][2], target_angle)
            start_angle = pos_message['self'][2]

            print(f"target angle: {target_angle}, current angle: {pos_message['self'][2]}, angle to move: {angle_to_move}")
            if type(angle_to_move) is type(np.array([0])):
                angle_to_move = angle_to_move[0]
            else:
                angle_to_move = angle_to_move

            # rotate
            if angle_to_move > 0:
                rot_l, rot_r = calculate_speed(0, opt.angular_speed) # self.speed_rk.solve_rotate_reverse(np.pi * 2)
            else:
                rot_l, rot_r = calculate_speed(0, -opt.angular_speed) # self.speed_rk.solve_rotate_reverse(-np.pi * 2)

            #rot_l, rot_r = 1500, -2000
            ## set wheel speed
            rot_l, rot_r = int(rot_l), int(rot_r)
            one_round = 360 / opt.angular_speed - opt.rotate_scale

            # move_time = abs(angle_to_move / opt.angular_speed)
            move_time = abs(angle_to_move / 360 * one_round) + 1
            custom_speed(rot_l, rot_r)

            # time.sleep(move_time + opt.rotate_scale * (360 / opt.angular_speed)) # 1.22  # wait for rotate while let other thread run
            # time.sleep(move_time)
            time.sleep(opt.rotate_scale)

            while abs(angle_diff(pos_message['self'][2], target_angle)) > 5:
                # print(f"current angle {pos_message['self'][2]}, target: {target_angle}, diff {angle_diff(pos_message['self'][2], target_angle)}")
                time.sleep(opt.update_time)

            # stop car
            stopcar()

            print(f"angle moved: {angle_diff(pos_message['self'][2], start_angle)}")

            # translate
    #            lin_l = self.speed_rk.solve_speed_reverse("left", 500)
    #            lin_r = self.speed_rk.solve_speed_reverse("right", 500)

            lin_l, lin_r = calculate_speed(400,0)
            #lin_l, lin_r = 2000, 2500 4
            ## set wheel speed
            lin_l, lin_r = int(lin_l), int(lin_r)
            move_time = target_dist / 400.
            custom_speed(lin_l, lin_r)

            turn_back = False
            # wait for move while let other thread run
            # time.sleep(move_time)  
            if sleep_time is not None:
                time.sleep(sleep_time)
                move_time = max(move_time - sleep_time, 0)

            for i in range(int(move_time / opt.update_time)):
                time.sleep(opt.update_time)
                # global pos_message
                near_dist, near_pos, near_key = self.calculate_near(pos_message)
                
                self_x, self_y = pos_message['self'][0], pos_message['self'][1]

                # print(f"nearst car: {near_key}, dist {near_dist}")

                if self.close_to_border(self_x, self_y):

                    print(f"close to border {self_x} {self_y}")
                    turn_back = True
                    next_x, next_y = self.center
                    break
                # avoid hitting another car
                elif near_dist < opt.minimal_dist:
                    print(f"cars too close")
                    print(f"nearst car: {near_key}, dist {near_dist}")
                    turn_back = True
                    other_x, other_y = near_pos

                    next_x = self_x + opt.away_scale * (self_x - other_x)
                    next_y = self_y + opt.away_scale * (self_y - other_y)
                    break

            # stop car
            stopcar()

            pos_message = {}

            time.sleep(1)

            if turn_back:
                self.move_to(next_x, next_y, 1)

    def calculate_near(self, positions):
        self_x = positions['self'][0]
        self_y = positions['self'][1]

        near_dist = 1e7
        near_pos = None
        near_key = None
        for k in positions.keys():
            if k == 'self':
                continue

            other_x, other_y = positions[k][0], positions[k][1]
            if math.isnan(other_x) or math.isnan(other_y):
                continue

            dist = np.linalg.norm(np.array([self_x, self_y]) - np.array([other_x, other_y]))

            if dist < near_dist:
                near_dist = dist
                near_pos = (other_x, other_y)
                near_key = k

        return near_dist, near_pos, near_key
    
    def close_to_border(self, self_x, self_y):
        if (self_x - self.x_low < opt.minimal_dist) or \
           (self_y - self.y_low < opt.minimal_dist) or \
           (self.x_high - self_x < opt.minimal_dist) or \
           (self.y_high - self_y < opt.minimal_dist):
            return True
        else:
            return False
        
    def map_position(self, x, y):
        return (self.center[0] + (x - 0.5) * self.x_span,
                self.center[1] + (y - 0.5) * self.y_span)

def main(args=None):
    rclpy.init(args=args)

    vicon_subscriber = ViconSubscriber()

    rclpy.spin(vicon_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vicon_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

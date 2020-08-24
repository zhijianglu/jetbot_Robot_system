#!/usr/bin/python3
from jetbot import Robot
import rospy
from std_msgs.msg import Float64MultiArray
import time

robot = Robot()


# moving control
def goforward(step_length, step_speed):
    global robot
    # robot = Robot()
    robot.forward(step_speed)  # Jetbot??
    time.sleep(step_length)
    robot.stop()


def gobackward(step_length, step_speed):
    global robot
    robot.backward(step_speed)  # Jetbot??
    time.sleep(step_length)
    robot.stop()


def goleft(step_length, step_speed):
    global robot
    robot.left(step_speed)
    time.sleep(step_length)
    robot.stop()


def goright(step_length, step_speed):
    global robot
    robot.right(step_speed)
    time.sleep(step_length)
    robot.stop()


def headup(step_length, step_speed):
    global robot
    robot.down(step_speed)
    time.sleep(step_length)
    robot.vertical_motors_stop()


def headdown(step_length, step_speed):
    global robot
    robot.up(step_speed)
    time.sleep(step_length)
    robot.vertical_motors_stop()


# cam control
# cam moving

def reset_cam():
    global servo_device
    servo_device.Servo_serial_double_control(1, 2100, 2, 2048)


def move_cam(c, h):
    global servo_device

    # prevent from over turn
    if c < 1300 or c > 3300:
        return

    if h < 1300 or h > 3300:
        return

    servo_device.Servo_serial_double_control(1, c, 2, h)


def look_on_road():
    global servo_device
    servo_device.Servo_serial_double_control(1, 2048, 2, 1900)


def look_rightleft(position):
    global servo_device
    if position < 1300 or position > 3300:
        return

    servo_device.Servo_serial_double_control(1, position)


def look_updown(position):
    if position < 1300 or position > 3300:
        print("out of value,cam will get stack")
        return

    global servo_device
    servo_device.Servo_serial_double_control(2, position)



# callback from ros msgs
def goforward_callback(msg):  # msgg???????,??????
    print('gofoward: step: %d,  speed:%d \n', msg.data[0], msg.data[1])
    goforward(msg.data[0], msg.data[1])


def gobackward_callback(msg):  # msgg???????,??????
    print('gobackward: step: %d,  speed:%d \n', msg.data[0], msg.data[1])
    gobackward(msg.data[0], msg.data[1])


def headdown_callback(msg):  # msgg???????,??????
    print('headdown: step: %d,  speed:%d \n', msg.data[0], msg.data[1])
    headdown(msg.data[0], msg.data[1])


def headup_callback(msg):  # msgg???????,??????
    print('headup: step: %d,  speed:%d \n', msg.data[0], msg.data[1])
    headup(msg.data[0], msg.data[1])


def turnleft_callback(msg):  # msgg???????,??????
    print('turnleft: step: %d,  speed:%d \n', msg.data[0], msg.data[1])
    goleft(msg.data[0], msg.data[1])


def turnright_callback(msg):  # msgg???????,??????
    print('turnright: step: %d,  speed:%d \n', msg.data[0], msg.data[1])
    goright(msg.data[0], msg.data[1])



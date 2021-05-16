#!/usr/bin/env python
import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

# Publisher for sending acceleration commands to flappy bird
pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)

# initialise errors for Derivative control
err_x_prev, err_y_prev = 0, 0
err_vx_prev, err_vy_prev = 0, 0

# set initial position from the start of the game
x_pos, y_pos = 0, -16
vx, vy = 0, 0

def initNode():
    # Here we initialize our node running the automation code
    rospy.init_node('flappy_automation_code', anonymous=True)

    # Subscribe to topics for velocity and laser scan from Flappy Bird game
    rospy.Subscriber("/flappy_vel", Vector3, velCallback)
    rospy.Subscriber("/flappy_laser_scan", LaserScan, laserScanCallback)

    # Ros spin to prevent program from exiting
    rospy.spin()

def velCallback(msg):
    """
    Callback to get velocity readings from the bird.
    (x,y) position  is also computed.
    :msg: has the format of geometry_msgs::Vector3
    """
    global vx, vy

    vx = msg.x
    vy = msg.y

    # Start the position estimator
    posEstimator()

def laserScanCallback(msg):
    """
    Callback to get range readings from the planar laser range-finder
    :msg: has the format of sensor_msgs::LaserScan
    """
    global ranges
    global primary_sum
    global upper_sum, lower_sum

    # normalize range values for ease of manipulation
    ranges = np.array(msg.ranges)/3.54999995

    # sum laser values of interest
    upper = ranges[[5,6,7,8]]
    lower = ranges[[0,1,2,3]]
    primary = ranges[[3,5]] # these two lasers will drive the bird

    upper_sum = sum(upper)
    lower_sum = sum(lower)
    primary_sum = sum(primary)

    # enable state machine
    stateMachine()

def posEstimator():
    """
    Compute (x,y) position by integrating velocity over loop iterations.
    """
    global x_pos, y_pos

    x_pos += vx
    y_pos += vy

def stateMachine():
    """
    State machine of the flappy bird.
    :state: ascend, descend, gothrough or adjust.
    """
    global state

    # reinitialize game cumulated values such as position in case the game is restarted
    reinitGame()

    # if one of the outer-middle ranges are too close, enable safety scan
    if (ranges[3] < 0.26 or ranges[5] < 0.26):
        if y_pos > 0 and state !='ascend':
            state = 'descend'
        elif y_pos < 0 and state !='descend':
            state = 'ascend'
        # set limits to not hit floor and ceiling
        if y_pos < -35:
            state = 'ascend'
        elif y_pos > 35:
            state = 'descend'
    # if all the middle ranges are large, a duct has been found
    elif (ranges[3] > 0.6 and ranges[4] > 0.6 and ranges[5] > 0.6):
        state = 'gothrough'
    # otherwise just use symmetric regulation + advance
    else:
        state = 'adjust'

    # enable controller
    controller()

def controller():
    """
    Cascaded control loop using a velocity PD-control setpoint fed through an
    acceleration PD-controller. Gains are set with setVelGains() and setAccGains() below.
    """
    global err_x_prev, err_y_prev
    global err_vx_prev, err_vy_prev

    if state == 'descend':
        err_x = -primary_sum # slow down
        err_y = -primary_sum # go down

        setVelGains(10, 3, 0.2, 0.3) # set x-y velocity PD gains
        setAccGains(10, 3, 0.3, 0.3) # set x-y acceleration PD gains

    elif state == 'ascend':
        err_x = -primary_sum # slow down
        err_y = primary_sum # go up

        setVelGains(10, 3, 0.2, 0.3)
        setAccGains(10, 3, 0.3, 0.3)

    elif state == 'gothrough':
        err_x = primary_sum # advance
        err_y = ranges[6] - ranges[2] # regulate for symmetric duct entry

        setVelGains(1, 1, 0.1, 0.2)
        setAccGains(30, 23, 2, 3)

    else: # adjust state
        err_x = primary_sum # advance
        err_y = upper_sum - lower_sum # adjust via imbalance to find holes

        setVelGains(1, 0.5, 0.1, 0.2)
        setAccGains(30, 23, 0.3, 1)

    # compute velocity input
    ref_vx = Kvx_p*err_x + Kvx_d*(err_x - err_x_prev)
    ref_vy = Kvy_p*err_y + Kvy_d*(err_y - err_y_prev)

    # compute difference between target and current velocity
    err_vx = ref_vx - vx
    err_vy = ref_vy - vy

    # compute acceleration input
    ref_ax = Kax_p*err_vx + Kax_d*(err_vx - err_vx_prev)
    ref_ay = Kay_p*err_vy + Kay_d*(err_vy - err_vy_prev)

    # save previous errors for Derivative control
    err_x_prev = err_x
    err_y_prev = err_y
    err_vx_prev = err_vx
    err_vy_prev = err_vy

    # send acceleration inputs
    sendAcc(ref_ax, ref_ay)

def setVelGains(_Kvx_p, _Kvy_p, _Kvx_d, _Kvy_d):
    """
    Set gains of velocity PD-controller.
    """
    global Kvx_p, Kvy_p # Proportional velocity gains
    global Kvx_d, Kvy_d # Derivative velocity gains

    Kvx_p, Kvy_p = _Kvx_p, _Kvy_p
    Kvx_d, Kvy_d = _Kvx_d, _Kvy_d

def setAccGains(_Kax_p, _Kay_p, _Kax_d, _Kay_d):
    """
    Set gains of acceleration PD-controller.
    """
    global Kax_p, Kay_p # Proportional acceleration gains
    global Kax_d, Kay_d # Derivative acceleration gains

    Kax_p, Kay_p = _Kax_p, _Kay_p
    Kax_d, Kay_d = _Kax_d, _Kay_d

def sendAcc(_ax, _ay):
    """
    Publish acceleration commands to /flappy_acc topic.
    :acc: has the format of geometry_msgs::Vector3
    """
    acc = Vector3(_ax, _ay, 0)
    pub_acc_cmd.publish(acc)

def reinitGame():
    """
    This is a function to reinitialize the cumulated values if the game was restarted without killing the script, since restarting the game while the node is still running does not reinitialize the script values.
    This function works by finding the specific initial configuration of the laser beams at the start of the game which has a sum of approx. 8.3.
    """
    global x_pos, y_pos
    global err_x_prev, err_y_prev
    global err_vx_prev, err_vy_prev

    if abs(sum(ranges) - 8.3) <= 0.2:
        x_pos, y_pos = 0, -16
        err_x_prev, err_y_prev = 0, 0
        err_vx_prev, err_vy_prev = 0, 0

if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass

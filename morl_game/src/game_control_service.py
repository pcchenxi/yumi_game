
#!/usr/bin/env python3
from morl_game.srv import *
import rospy
from geometry_msgs.msg import WrenchStamped
import numpy as np

sensor_left, sensor_right = [], []

def control_game(req):
    print('request type', req.cmd_type, sensor_left)
    response = req.cmd_data

    obs = np.concatenate((sensor_left, sensor_right), axis=0)
    return GameCmdResponse(obs)


def callback_left(data):
    global sensor_left
    sensor_reading = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z]
    sensor_left = sensor_reading
    # print('left', sensor_reading)
    # rospy.loginfo('left', rospy.get_caller_id() + "I heard %s", data.data)

def callback_right(data):
    global sensor_right
    sensor_reading = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z]
    sensor_right = sensor_reading
    # print('right', sensor_reading)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)    

if __name__ == "__main__":
    rospy.init_node('game_control_server')
    s = rospy.Service('game_control', GameCmd, control_game)
    print ("Ready to interact with game environment.")

    rospy.Subscriber("/left_sensor/ethdaq_data", WrenchStamped, callback_left)
    rospy.Subscriber("/right_sensor/ethdaq_data", WrenchStamped, callback_right)

    rospy.spin()
#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

import flask
import numpy as np
from scipy.spatial.transform import Rotation
from arm_tutorial.kinematics_util import get_rotate_x


app = flask.Flask(__name__)
driver = None

class Kinematics3D(Node):
    def __init__(self):
        super().__init__('kinematics_3d')

        # joint states publisher
        self.pub_joint_states = self.create_publisher(JointState, "joint_states", 10)

        # init variable
        self.joint_states = JointState()
        self.joint_states.header.frame_id = "joint_states"
        self.joint_states.name = ["body_to_joint01", "link01_to_joint02", "link02_to_joint03"]
        self.joint_states.position = [0.0, 0.0, 0.0]

        # timer
        self.timer = self.create_timer(0.1, self.publish_callback)
    
    def publish_callback(self):
        curr_time = self.get_clock().now()

        # joint states
        self.joint_states.header.stamp = curr_time.to_msg()

        # publish
        self.pub_joint_states.publish(self.joint_states)


def init_ros(args=None):
    global driver
    rclpy.init(args=args)

    driver = Kinematics3D()
    rclpy.spin(driver)

    driver.destroy_node()
    rclpy.shutdown()


@app.route('/', methods=['GET'])
def get_states():
    return flask.jsonify(make_data())


@app.route('/', methods=['PUT'])
def set_states():
    data = flask.request.get_json()
    for i, v in enumerate(data['joint_states']):
        driver.joint_states.position[i] = float(v)
    return flask.jsonify(make_data())


def main(args=None):
    threading.Thread(target=lambda: init_ros(args)).start()
    app.run(host='0.0.0.0', port=5000)


def make_data():
    d01 = driver.joint_states.position[0]
    d02 = driver.joint_states.position[1]
    d03 = driver.joint_states.position[2]

    p01 = np.array([0.05, 0.0, 0.15])
    p12 = np.array([-0.05, 0.0, 0.45])
    p23 = np.array([0.05, 0.0, 0.3])
    p3e = np.array([0.0, 0.0, 0.2])

    R01 = get_rotate_x(d01)
    R12 = get_rotate_x(d02)
    R23 = get_rotate_x(d03)

    g01 = p01
    g02 = g01 + R01.dot(p12)
    g03 = g02 + R01.dot(R12).dot(p23)
    g0e = g03 + R01.dot(R12).dot(R23).dot(p3e)

    r01 = Rotation.from_euler('x', d01)
    r02 = r01 * Rotation.from_euler('x', d02)
    r03 = r02 * Rotation.from_euler('x', d03)
    r0e = r03

    data = {
        'joint_states': list(driver.joint_states.position),
        'joint_01': {
            'position': list(np.round(g01[:3], 6)),
            'orientation': list(np.round(r01.as_quat(), 6)),
        },
        'joint_02': {
            'position': list(np.round(g02[:3], 6)),
            'orientation': list(np.round(r02.as_quat(), 6)),
        },
        'joint_03': {
            'position': list(np.round(g03[:3], 6)),
            'orientation': list(np.round(r03.as_quat(), 6)),
        },
        'edge': {
            'position': list(np.round(g0e[:3], 6)),
            'orientation': list(np.round(r0e.as_quat(), 6)),
        }
    }
    return data


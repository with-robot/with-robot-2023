#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

import flask
import numpy as np
from scipy.spatial.transform import Rotation
from arm_tutorial.kinematics_util import get_rotate_x, get_rotate_z


app = flask.Flask(__name__)
driver = None

class Kinematics6D(Node):
    def __init__(self):
        super().__init__('kinematics_6d')

        # joint states publisher
        self.pub_joint_states = self.create_publisher(JointState, "joint_states", 10)

        # init variable
        self.joint_states = JointState()
        self.joint_states.header.frame_id = "joint_states"
        self.joint_states.name = ["body_to_joint01",
                                  "link01_to_joint02",
                                  "link02_to_joint03",
                                  "link03_to_joint04",
                                  "link04_to_joint05",
                                  "link05_to_joint06"]
        self.joint_states.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

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

    driver = Kinematics6D()
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
    d04 = driver.joint_states.position[3]
    d05 = driver.joint_states.position[4]
    d06 = driver.joint_states.position[5]

    p01 = np.array([0.0, 0.0, 0.225])
    p12 = np.array([0.125, 0.0, 0.0])
    p23 = np.array([0.0, 0.0, 0.525])
    p34 = np.array([-0.125, 0.0, 0.0])
    p45 = np.array([0.0, 0.0, 0.375])
    p56 = np.array([0.125, 0.0, 0.0])
    p6e = np.array([0.0, 0.0, 0.2])

    R01 = get_rotate_z(d01)
    R12 = get_rotate_x(d02)
    R23 = get_rotate_z(d03)
    R34 = get_rotate_x(d04)
    R45 = get_rotate_z(d05)
    R56 = get_rotate_x(d06)

    g01 = p01
    g02 = g01 + R01.dot(p12)
    g03 = g02 + R01.dot(R12).dot(p23)
    g04 = g03 + R01.dot(R12).dot(R23).dot(p34)
    g05 = g04 + R01.dot(R12).dot(R23).dot(R34).dot(p45)
    g06 = g05 + R01.dot(R12).dot(R23).dot(R34).dot(R45).dot(p56)
    g0e = g06 + R01.dot(R12).dot(R23).dot(R34).dot(R45).dot(R56).dot(p6e)

    r01 = Rotation.from_euler('z', d01)
    r02 = r01 * Rotation.from_euler('x', d02)
    r03 = r02 * Rotation.from_euler('z', d03)
    r04 = r03 * Rotation.from_euler('x', d04)
    r05 = r04 * Rotation.from_euler('z', d05)
    r06 = r05 * Rotation.from_euler('x', d06)
    r0e = r06

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
        'joint_04': {
            'position': list(np.round(g04[:3], 6)),
            'orientation': list(np.round(r04.as_quat(), 6)),
        },
        'joint_05': {
            'position': list(np.round(g05[:3], 6)),
            'orientation': list(np.round(r05.as_quat(), 6)),
        },
        'joint_06': {
            'position': list(np.round(g06[:3], 6)),
            'orientation': list(np.round(r06.as_quat(), 6)),
        },
        'edge': {
            'position': list(np.round(g0e[:3], 6)),
            'orientation': list(np.round(r0e.as_quat(), 6)),
        }
    }
    return data


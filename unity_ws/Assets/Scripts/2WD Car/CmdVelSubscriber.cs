using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.Core;

public class CmdVelSubscriber : MonoBehaviour
{
    [SerializeField] private string topicName = "cmd_vel";
    [SerializeField] private float publishFrequency = 0.1f;

    [SerializeField] private float motorForce;
    [SerializeField] private float horizontalRate;
    [SerializeField] private WheelCollider leftWheelCollider;
    [SerializeField] private WheelCollider rightWheelCollider;

    ROSConnection ros;
    private float timeElapsed = 0.0f;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(topicName, CmdVelCallback);
    }

    private void CmdVelCallback(TwistMsg msg)
    {
        leftWheelCollider.motorTorque = (float)(msg.linear.x - msg.angular.z * horizontalRate) * motorForce;
        rightWheelCollider.motorTorque = (float)(msg.linear.x + msg.angular.z * horizontalRate) * motorForce;
    }
}

using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.Core;

public class VelRawPublisher : MonoBehaviour
{
    [SerializeField] private string topicName = "vel_raw";
    [SerializeField] private float publishFrequency = 0.1f;

    ROSConnection ros;
    private float timeElapsed = 0.0f;

    private TwistStampedMsg vel_raw;
    private Vector3 prev_position;
    private Quaternion prev_rotation;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistStampedMsg>(topicName);

        vel_raw = new TwistStampedMsg();
    }

    private void FixedUpdate()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed >= publishFrequency)
        {
            if (prev_position == null || prev_rotation == null)
            {
                prev_position = transform.position;
                prev_rotation = transform.rotation;
                timeElapsed = 0;
                return;
            }
            // now timestamp
            var now = Clock.Now;
            var stamp = new TimeMsg
            {
                sec = (int)now,
                nanosec = (uint)((now - Math.Floor(now)) * Clock.k_NanoSecondsInSeconds)
            };
            // cal val
            Vector3 distance = transform.position - prev_position;
            float angle = Quaternion.Angle(transform.rotation, prev_rotation);
            // cal vel_raw
            vel_raw.twist.linear.x = distance.z / timeElapsed;
            vel_raw.twist.linear.y = 0;
            vel_raw.twist.angular.z = angle * Mathf.Deg2Rad / timeElapsed;
            // init values
            prev_position = transform.position;
            prev_rotation = transform.rotation;
            timeElapsed = 0;
            // publish ros topic
            vel_raw.header.stamp = stamp;
            ros.Publish(topicName, vel_raw);
        }
    }
}

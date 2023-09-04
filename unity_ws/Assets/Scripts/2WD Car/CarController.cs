using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarController : MonoBehaviour
{

    private const string HORIZONTAL = "Horizontal";
    private const string VERTICAL = "Vertical";

    private float horizontalInput;
    private float verticalInput;

    [SerializeField] private float motorForce;
    [SerializeField] private float horizontalRate;

    [SerializeField] private WheelCollider leftWheelCollider;
    [SerializeField] private WheelCollider rightWheelCollider;

    private void FixedUpdate()
    {
        GetInput();
        HandleMotor();
    }

    private void GetInput()
    {
        horizontalInput = Input.GetAxis(HORIZONTAL);
        verticalInput = Input.GetAxis(VERTICAL);
    }

    private void HandleMotor()
    {
        leftWheelCollider.motorTorque = (verticalInput + horizontalInput * horizontalRate) * motorForce;
        rightWheelCollider.motorTorque = (verticalInput - horizontalInput * horizontalRate) * motorForce;
    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/* This class can be used to define all the specific hardware for the Innovators Robot, InnoBot. */

abstract public class InnoBot_SensorTest
{

     /* SENSORS */
    public OpticalDistanceSensor opticalDistanceSensor = null;
    public TouchSensor touchSensor = null;
    public GyroSensor gyroSensor = null;

    public ColorSensor colorSensor_floor = null;
    public ColorSensor colorSensor_beacon = null;
    public ColorSensor colorSensor_ball = null;

    public static final double CSERVO_STOP       =  0.5 ;
    public static final double CSERVO_CW = 0.0;
    public static final double CSERVO_CCW = 1.0;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    double count = 0;
    boolean drivingMode;

    /* Constructor */
    public InnoBot_SensorTest(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        drivingMode = false;

        /****************************************
         * MOTORS/SERVOS/SENSORS
         * ****************************************/

         /* SENSORS */
        opticalDistanceSensor = hwMap.opticalDistanceSensor.get("ODS");

        touchSensor = hwMap.touchSensor.get("Touch Sensor");

        colorSensor_beacon = hwMap.colorSensor.get("colorSensor_Beacon");
        colorSensor_beacon.enableLed(true); // Set the LED on in the beginning

        colorSensor_floor = hwMap.colorSensor.get("colorSensor_Floor");
        colorSensor_floor.enableLed(true);  // Set the LED on in the beginning

        colorSensor_ball = hwMap.colorSensor.get("colorSensor_Ball");
        colorSensor_ball.enableLed(true);  // Set the LED on in the beginning */

        gyroSensor = hwMap.gyroSensor.get("GS");

    }


    /**************************************************
     * Abstract Methods to be Created By Team Members *
     * @param power
     ***************************************************/
    abstract public void driveForwardAtPowerUntilWhiteLineDetectedOnFloor(double power, Telemetry telemetry);
    abstract public void runBallShooterMotorIfBlueBeaconDetected(Telemetry telemetry);
    abstract public void runBeaconPusherServoIfRedBeaconDetected(Telemetry telemetry);
    abstract public void runBeaconPusherServoIfBlueBeaconDetected(Telemetry telemetry);
    abstract public void rotateToFaceAngle(double targetAngleInDegrees, Telemetry telemetry);
    abstract public void rotateAngleDegrees(double angleToRotate, Telemetry telemetry);


}
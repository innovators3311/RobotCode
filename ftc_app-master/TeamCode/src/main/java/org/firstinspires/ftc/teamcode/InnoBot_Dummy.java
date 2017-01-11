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

abstract public class InnoBot_Dummy
{
    /* Public OpMode members. */
    /****************************************
     * MOTORS/SERVOS/SENSORS
     * ****************************************/

    /* TASK MOTORS */
    public DcMotor  ballTransfer = null;
    public DcMotor  ballPickup  = null;
    public DcMotor  ballShooter = null;

    /* SERVO MOTORS */
    public Servo beaconPusher = null;

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
    public InnoBot_Dummy(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        drivingMode = false;

        /****************************************
         * MOTORS/SERVOS/SENSORS
         * ****************************************/

        /* TASK MOTORS */
        ballTransfer = hwMap.dcMotor.get("ballTransfer");
        ballShooter = hwMap.dcMotor.get("ballShooter");

        /* SERVO MOTORS */
        //servo_ThrowerAngle = hwMap.servo.get("servo_ThrowerAngle");
        beaconPusher = hwMap.servo.get("beaconPusher");

        colorSensor_beacon = hwMap.colorSensor.get("colorSensor_Beacon");
        colorSensor_beacon.enableLed(true); // Set the LED on in the beginning

        gyroSensor = hwMap.gyroSensor.get("GS");


        // Set all task motors to zero power to start
        ballShooter.setPower(0);
        ballTransfer.setPower(0);

        // Define and initialize ALL installed servos.
        //servo_ThrowerAngle.setPosition(MID_SERVO);
        beaconPusher.setPosition(CSERVO_STOP);
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

    /*******************************************************************/
    /* TASK METHODS */
    /*******************************************************************/
    public void shooterOn() {
        ballShooter.setPower(1.0);
    }
    final public void shooterOff() { ballShooter.setPower(0.0); }
    final public void pickupOn() { ballPickup.setPower(1.0); }
    final public void pickupOff() { ballPickup.setPower(0.0); }
    final public void transferOn() {
        ballTransfer.setPower(1.0);
    }
    final public void transferOff() {
        ballTransfer.setPower(0.0);
    }
}
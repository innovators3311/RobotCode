package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/* This class can be used to define all the specific hardware for the Innovators Robot, InnoBot. */

abstract public class InnoBot
{
    /* Public OpMode members. */
    /****************************************
     * MOTORS/SERVOS/SENSORS
     * ****************************************/
    /* DRIVING MOTORS */
    public DcMotor  leftFrontWheel   = null;
    public DcMotor  rightFrontWheel  = null;
    public DcMotor  leftBackWheel   = null;
    public DcMotor  rightBackWheel  = null;

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
    public InnoBot(){

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
        ballPickup  = hwMap.dcMotor.get("ballPickup");
        ballTransfer = hwMap.dcMotor.get("ballTransfer");
        ballShooter = hwMap.dcMotor.get("ballShooter");

        /* SERVO MOTORS */
        //servo_ThrowerAngle = hwMap.servo.get("servo_ThrowerAngle");
        beaconPusher = hwMap.servo.get("beaconPusher");

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


        /* DRIVING MOTORS */
        /* Initialize to correct instance in physical robot configuration */
        leftFrontWheel   = hwMap.dcMotor.get("LF");
        rightFrontWheel  = hwMap.dcMotor.get("RF");
        leftBackWheel   = hwMap.dcMotor.get("LB");
        rightBackWheel  = hwMap.dcMotor.get("RB");
        // Set direction based on whether using AndyMark motors or not.
        leftFrontWheel.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontWheel.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftBackWheel.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightBackWheel.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Set all wheel motors to zero power to start
        leftFrontWheel.setPower(0);
        rightFrontWheel.setPower(0);
        leftBackWheel.setPower(0);
        rightBackWheel.setPower(0);
        // Set all task motors to zero power to start
        ballPickup.setPower(0);
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
    abstract public void runBeaconPusherServoIfRedBeaconDetected(Telemetry telemetry);
    abstract public void runBeaconPusherServoIfBlueBeaconDetected(Telemetry telemetry);
    abstract public void rotateToFaceAngle(double targetAngleInDegrees, Telemetry telemetry);
    abstract public void rotateAngleDegrees(double angleToRotate, Telemetry telemetry);

    /*******************************************************************/
    /* DRIVING METHODS */
    /*******************************************************************/
    final public void driveForward() {
        driveForwardAtPower(1);
    }
    final public void driveBackward() {
        driveBackwardAtPower(1);
    }
    final public void driveForwardAtPower(double power) {
        setAllWheelsToPower(power);
    }
    final public void driveBackwardAtPower(double power) {
        driveForwardAtPower(-power);
    }
    final public void stopDriving() {
        setAllWheelsToPower(0);
    }
    final public void stopWheels() {
        setAllWheelsToPower(0);
    }

    final public void drive(double leftPower, double rightPower) {
        rightFrontWheel.setPower(rightPower);
        rightBackWheel.setPower(rightPower);
        leftBackWheel.setPower(leftPower);
        leftFrontWheel.setPower(leftPower);
    }

    final public void setAllWheelsToPower(double power) {
        rightFrontWheel.setPower(power);
        rightBackWheel.setPower(power);
        leftBackWheel.setPower(power);
        leftFrontWheel.setPower(power);
    }

    /*******************************************************************/
    /* STRAFING METHODS */
    /*******************************************************************/
    final public void strafeRight(double power) { strafeLeft(-power); }

    final public void strafeLeft(double power) {
        rightFrontWheel.setPower(power);
        rightBackWheel.setPower(-power);
        leftFrontWheel.setPower(-power);
        leftBackWheel.setPower(power);
    }


    /*******************************************************************/
    /* SPINNING METHODS */
    /*******************************************************************/
    final public void spinCW() {
        spinCWAtPower(1);
    }
    final public void spinCCW() { spinCWAtPower(-1); }
    final public void stopSpinning() {
        stopDriving();
    }
    final public void spinCCWAtPower(double power) { spinCWAtPower(-power); }

    final public void spinCWAtPower(double power) {
        rightFrontWheel.setPower(-power);
        rightBackWheel.setPower(-power);
        leftBackWheel.setPower(power);
        leftFrontWheel.setPower(power);
    }


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

    final public void setWheelMode(DcMotor.RunMode runMode) {

        leftFrontWheel.setMode(runMode);
        rightFrontWheel.setMode(runMode);
        leftBackWheel.setMode(runMode);
        rightBackWheel.setMode(runMode);
    }
}
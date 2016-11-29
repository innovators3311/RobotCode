package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left front drive motor:        "LF"
 * Motor channel:  Right front drive motor:       "RF"
 * Motor channel:  Left back drive motor:         "LB"
 * Motor channel:  Right back drive motor:        "RB"
 */
public class HardwareInnovatorBot
{
    /* Public OpMode members. */
    public DcMotor  leftFrontMotor   = null;
    public DcMotor  rightFrontMotor  = null;
    public DcMotor  leftBackMotor   = null;
    public DcMotor  rightBackMotor  = null;
    public Servo servo1 = null;
    public DcMotor  pickupMotor  = null;
    public OpticalDistanceSensor odsSensor;
    public TouchSensor touchSensor;
    public ColorSensor colorSensor;
    public GyroSensor gyroSensor;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareInnovatorBot(){

    }

    public void jen_strafeLeftAtPowerForDuration(float power, double durationInSeconds) {
        ElapsedTime     runtime = new ElapsedTime();

        strafeLeft(power);
        runtime.reset();
        while (runtime.seconds() < durationInSeconds) {

        }
        stop();
    }

    public void drive(double left, double right) {
        rightFrontMotor.setPower(right);
        rightBackMotor.setPower(right);
        leftBackMotor.setPower(left);
        leftFrontMotor.setPower(left);
    }

    public void driveForwardAtPower(double power) {
        drive(power, power);
    }

    public void driveBackwardAtPower(double power) {
        drive(-power, -power);
    }

    public void stop() {
        drive(0,0);
    }

    public void strafeLeft(double power) {
        rightFrontMotor.setPower(-power);
        rightBackMotor.setPower(power);
        leftBackMotor.setPower(-power);
        leftFrontMotor.setPower(power);
    }

    public void strafeRight(double power) {
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(-power);
        leftBackMotor.setPower(power);
        leftFrontMotor.setPower(-power);
    }

    public void driveForwardAtPowerForDuration(double power, double duration) {
        ElapsedTime     runtime = new ElapsedTime();

        drive(power, power);
        runtime.reset();
        while (runtime.seconds() < duration) {

        }
        stop();
    }

    public void spinCCWAtPowerForDuration(double power, double duration) {
        ElapsedTime     runtime = new ElapsedTime();
        spinCCW(power);
        runtime.reset();
        while (runtime.seconds() < duration) {

        }
        stop();
    }

    public void spinCCW(double power) {
        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(-power);
        leftBackMotor.setPower(power);
        rightBackMotor.setPower(-power);
    }

    public void spinCW(double power) {
        leftFrontMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        leftBackMotor.setPower(-power);
        rightBackMotor.setPower(power);
    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontMotor   = hwMap.dcMotor.get("LF");
        rightFrontMotor  = hwMap.dcMotor.get("RF");
        leftBackMotor   = hwMap.dcMotor.get("LB");
        rightBackMotor  = hwMap.dcMotor.get("RB");
        pickupMotor  = hwMap.dcMotor.get("Pickup1");

        // Define and Initialize Motors
        odsSensor = hwMap.opticalDistanceSensor.get("ODS");
        touchSensor = hwMap.touchSensor.get("Touch Sensor");
        colorSensor = hwMap.colorSensor.get("CS");
        gyroSensor = hwMap.gyroSensor.get("GS");


        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        pickupMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        pickupMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pickupMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Define and initialize ALL installed servos.
        servo1 = hwMap.servo.get("S1");
        servo1.setPosition(MID_SERVO);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

    /* Sensor Information
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Optical Distance Sensor (45-2006)
    The Optical Distance Sensor (ODS) is an analog sensor that uses electro optical proximity detection
    to calculate distance from an object based on the intensity of the light. This sensor can accurately
    calculate distances between 1 cm and 15 cm. Lighter colored objects will return a more accurate and
    consistent reading. Try different colors and materials to see what works best for you. The ODS can
    be used for object detection, line detection and the difference between light and dark.

    http://modernroboticsinc.com/optical-distance-sensor-2

    Sensor Type: Three Wire Analog
    Sensor Dimensions: 32 x 32 x 12 millimeters
    Mounting Holes: 24 x 24 millimeters
    Power: 5 volts DC, 20 mA max.
    Signal Logic Levels: Analog 0 – 5 volts


    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Touch Sensor (45-2007)

    The Touch Sensor can be used for an array of different tasks including object detection, counter,
    standard push button and many more. The sensor can be attached to either an analog or digital
    port and contains a built in LED which indicates when the sensor is activated.

    http://modernroboticsinc.com/touch-sensor-2

    Sensor Type: Three Wire Digital Sensor
    Dimensions: 36 x 32 x 15 millimeters
    Mounting Holes: 24 x 24 millimeters
    Power: 5 volts DC, 20 mA max.
    Signal Logic Levels: Logic 0 – 0 volts, Logic 1 – 5 volts
    Actuator Length: 5 millimeters
    Actuator Depression Force: 150 gram

    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Color Sensor (45-2018)

    The Color Sensor is used to read the color of an object and returns a handful of useful data using
    a red/green/blue reading. This data includes a color number that corresponds to the color line in
    the documentation, as well as raw and adjusted readings. The material of the surface being read and
    the ambient light in the room will affect the results. Therefore the Color Sensor should be recalibrated
    for different environments. This sensor has a maximum distance of 7cm.

    http://modernroboticsinc.com/color-sensor

    Sensor Type: Four Wire I2C Sensor
    Default I2C Address: 0x3C
    Dimensions: 32 x 32 x 11 millimeters
    Mounting Holes: 24 x 24 millimeters
    Power: 5 volts DC, 20 mA max.
    Signal Logic Levels: Logic 0 – 0 volts, Logic 1 – 5 volts I2C
    Bus Speed: 100kHz max I2C
    Address Change Option: yes

    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Gyro Sensor (45-2005)

    The Modern Robotics Integrating Gyro sensor reads rate of rotation around the X, Y and Z axes.  The sensor
    also calculates a heading based on movment around the Z axis.  The heading value is great for navigation and
    shows the current heading in degrees from 0 - 359 from a point where the heading value was reset to zero. 
    The heading is calculated in the sensor by the onboard microcontroller which reads the gyro sensor Z axis output
    approximately 760 times per second.  Performing the heading calculations within the sensor provides much greater
    accuracy than can be achieved if the calculation was done by the host. The heading value can be reset to zero
    at any time.

    There is an LED in the sensor that blinks at 1Hz during normal operation and will remain on during a null \
    and reset operations.

    Default I2C address 0x20 (0x10 7-bit).
    */

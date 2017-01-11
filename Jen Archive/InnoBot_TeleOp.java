/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides basic TeleOp driving for the Innovators 3311 robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common InnoBot hardware class to define the devices on the robot.
 * All device access is managed through the InnoBot class.
 *
 */

@TeleOp(name="InnoBot_TeleOp", group="InnoBot")
@Disabled
public class InnoBot_TeleOp extends OpMode{

    private static final double CSERVO_STOP = 0.5;
    /* Declare OpMode members. */
    InnoBot robot = new InnoBot_Basic();

    final double    CSERVO_SPEED  = 0.02 ;
    double          DRIVE_POWER = 1.0;

    /*************************************************
     * COLOR SENSOR
     * INITIALIZE VARIABLES USED BY COLOR SENSOR
     * Copy and paste this entire block above the loop in any file using the colorSensor
     *************************************************/
    // bPrevState and bCurrState represent the previous and current state of the button.
    boolean bColorSensor_LedPrevState = false;
    boolean bColorSensor_LedCurrState = false;

    // bLedOn represents the state of the LED.
    boolean bColorSensor_LedOn = true;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Innovators 3311 Driver!");    //
/*
        // Turn on color sensor LEDs
        telemetry.addData("init>", "INITIALIZING ROBOT");
        telemetry.addData("init>", "Turn on Color Sensor LEDs.");
        robot.colorSensor_floor.enableLed(true);
        robot.colorSensor_beacon.enableLed(true);
        // robot.colorSensor_ball.enableLed(bColorSensor_LedOn);

        // start calibrating the gyro.
        telemetry.addData("init>", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        robot.gyroSensor.calibrate();
        // make sure the gyro is calibrated.
        while (robot.gyroSensor.isCalibrating())  {

        }

        telemetry.addData("init>", "Set Servo to Start Positions");
        robot.beaconPusher.setPosition(CSERVO_STOP);
        //robot.servo_ThrowerAngle.setPosition(MID_SERVO);

        telemetry.addData("init>", "Gyro Calibrated.  Press Play Button when Ready.");
        telemetry.update();*/
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        /********************************************************************************************************************/
        // GAMEPAD 1 - DRIVING CONTROLLER
        /********************************************************************************************************************/
        // GAMEPAD BUMPERS - DEPRESSED INDICATES STRAFING LEFT OR RIGHT
        // RIGHT BUMPER - Strafe right
        // LEFT BUMPER - Strafe left

        boolean bLedOn = false;
        double currentHeading = 0.0;
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};
        double left_x = Range.clip(-gamepad1.left_stick_x, -1, 1); // double left = value from gamepad1 left joystick UP;
        double left_y = Range.clip(-gamepad1.left_stick_y, -1, 1); // double left = value from gamepad1 left joystick UP;
        double right_x = Range.clip(-gamepad1.right_stick_x, -1, 1); // double right = value from gamepad1 right joystick UP;
        double right_y = Range.clip(-gamepad1.right_stick_y, -1, 1); // double right = value from gamepad1 right joystick UP;

        double threshold = 0.30; // when is the joystick pushed enough that it is noticed

        if (gamepad1.a) {
            robot.drivingMode = true;
        }
        if (gamepad1.b) {
            robot.drivingMode = false;
        }


        if (gamepad1.left_bumper) {
            robot.strafeLeft(DRIVE_POWER);

        } else if (gamepad1.right_bumper) {
            robot.strafeRight(DRIVE_POWER);
        } else {

            if (robot.drivingMode) {

                if (Math.abs(left_y) > threshold) {
                    robot.driveForwardAtPower(-left_y);
                } else if (Math.abs(left_x) > threshold) {
                    robot.spinCWAtPower(left_x);
                } else {
                    if (gamepad1.dpad_up) { // DRIVE FORWARD
                        robot.driveForward();
                    } else if (gamepad1.dpad_down) { // DRIVE BACKWARD
                        robot.driveBackward();
                    } else if (gamepad1.dpad_left) { // SPIN CCW
                        robot.spinCW();
                    } else if (gamepad1.dpad_right) { // SPIN CW
                        robot.spinCCW();
                    } else {
                        robot.stopDriving();
                    }
                }

            } else {


                if (Math.abs(left_y) > threshold || Math.abs(left_x) > threshold) {
                    double r = Math.hypot(left_x, left_y);
                    double robotAngle = Math.atan2(left_y, left_x) - Math.PI / 4;
                    final double v1 = r * Math.cos(robotAngle) + right_x;
                    final double v2 = r * Math.sin(robotAngle) - right_x;
                    final double v3 = r * Math.sin(robotAngle) + right_x;
                    final double v4 = r * Math.cos(robotAngle) - right_x;

                    robot.leftFrontWheel.setPower(v1);
                    robot.rightFrontWheel.setPower(v2);
                    robot.leftBackWheel.setPower(v3);
                    robot.rightBackWheel.setPower(v4);

                } else {  // USE DPAD TO TEST DRIVING METHODS FROM InnoBot.java
                    if (gamepad1.dpad_up) { // DRIVE FORWARD
                        robot.driveForward();
                    } else if (gamepad1.dpad_down) { // DRIVE BACKWARD
                        robot.driveBackward();
                    } else if (gamepad1.dpad_left) { // SPIN CCW
                        robot.spinCW();
                    } else if (gamepad1.dpad_right) { // SPIN CW
                        robot.spinCCW();
                    } else {
                        robot.stopDriving();
                    }
                }
            }
        }

        currentHeading = robot.gyroSensor.getHeading();
        robot.colorSensor_floor.enableLed(true);

        telemetry.addData("GYRO:", "Current Heading: " + currentHeading);
        // send the info back to driver station using telemetry function.
        telemetry.addData("Floor Clear", robot.colorSensor_floor.alpha());
        telemetry.addData("Floor Red  ", robot.colorSensor_floor.red());
        telemetry.addData("Floor Green", robot.colorSensor_floor.green());
        telemetry.addData("Floor Blue ", robot.colorSensor_floor.blue());

        robot.colorSensor_beacon.enableLed(true);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Beacon Clear", robot.colorSensor_beacon.alpha());
        telemetry.addData("Beacon Red  ", robot.colorSensor_beacon.red());
        telemetry.addData("Beacon Green", robot.colorSensor_beacon.green());
        telemetry.addData("Beacon Blue ", robot.colorSensor_beacon.blue());
        updateTelemetry(telemetry);


        //********************************************************************************************************************/
        // GAMEPAD 2 - TOOL CONTROLLER
        //********************************************************************************************************************/

        // GAMEPAD BUMPERS - RUN BALL PICKUP MOTOR
        // RIGHT BUMPER - Run ball pickup motor FORWARD to collect balls
        // LEFT BUMPER - Run ball pickup motor BACKWARD to reject balls

        if (gamepad2.right_bumper) {
            robot.ballPickup.setPower(1);
        } else if (gamepad2.left_bumper) {
            robot.ballPickup.setPower(-1);
        } else {
            robot.ballPickup.setPower(0);
        }

        // GAMEPAD TRIGGERS - RUN BALL PICKUP MOTOR
        // RIGHT TRIGGER - Run ball transfer motor FORWARD to collect balls
        // LEFT TRIGGER - Run ball transfer motor BACKWARD to reject balls

        if (gamepad2.left_trigger > 0.0) {
            robot.ballTransfer.setPower(1);
        } else if (gamepad2.right_trigger > 0.0) {
            robot.ballTransfer.setPower(-1);
        } else {
            robot.ballTransfer.setPower(0);
        }

        // GAMEPAD TRIGGERS - RUN BALL PICKUP MOTOR
        // RIGHT TRIGGER - Run ball transfer motor FORWARD to collect balls
        // LEFT TRIGGER - Run ball transfer motor BACKWARD to reject balls

        if (gamepad2.y) {
            robot.ballShooter.setPower(1.0);
        } else {
            robot.ballShooter.setPower(0);
        }

        if (gamepad2.dpad_left) {
            robot.beaconPusher.setPosition(1.0);
        } else if (gamepad2.dpad_right) {
            robot.beaconPusher.setPosition(0.0);
        } else {
            robot.beaconPusher.setPosition(0.5);
        }

    } // end of loop()




    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}

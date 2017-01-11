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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - InnoBot_Basic forward for 48 inches
 *   - Spin right for 12 Inches
 *   - InnoBot_Basic Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoDrive_Basic", group="InnoBot")
//@Disabled
public class InnoBot_AutoDrive_Basic extends LinearOpMode {

    /* Declare OpMode members. */
    InnoBot         robot   = new InnoBot_Basic();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.setWheelMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        idle();

        robot.setWheelMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runBeaconPusherAtPowerForSeconds(-1.0, 1.5);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at Left %7d :%7d",
                          robot.leftFrontWheel.getCurrentPosition(),
                          robot.leftBackWheel.getCurrentPosition());
        telemetry.addData("Path0",  "Starting at Right %7d :%7d",
                robot.rightFrontWheel.getCurrentPosition(),
                robot.rightBackWheel.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        /* DISABLED SO XANDER AND TED CAN WORK ON SHOOTER
        spinUpBallShooterToPowerForSeconds(1.0, 2.0);
        robot.ballShooter.setPower(1.0);
        runBallTransferAtPowerForSeconds(1.0, 4.0);
        robot.ballShooter.setPower(0.0);
        robot.ballPickup.setPower(0.0);
        */

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  3,  3, 5.0);  // S1: Forward 3 Inches with 5 Sec timeout

        encoderDrive(TURN_SPEED,   -2, 2, 4.0);  // S2: Turn Right 2 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 20, 20, 4.0);  // S3: Forward 20 Inches with 4 Sec timeout

        encoderDrive(TURN_SPEED,   6, -6, 4.0);  // S2: Turn Left 6 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 20, 20, 4.0);  // S3: Forward 20 Inches with 4 Sec timeout

        encoderDrive(TURN_SPEED,   6, -6, 4.0);  // S2: Turn Left 6 nches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 20, 20, 4.0);  // S3: Forward 20 Inches with 4 Sec timeout

        encoderDrive(TURN_SPEED,   6, -6, 4.0);  // S2: Turn Right 6 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 20, 20, 4.0);  // S3: Forward 20 Inches with 4 Sec timeout

        /*encoderStrafe(DRIVE_SPEED, 3, 3, 5.0);*/

        runBeaconPusherAtPowerForSeconds(1.0, 2.0);
        runBeaconPusherAtPowerForSeconds(-1.0, 2.0);
        encoderDrive(DRIVE_SPEED, -20,-20, 4.0);  // S3: Forward 24 Inches with 4 Sec timeout
        runBeaconPusherAtPowerForSeconds(1.0, 2.0);
        runBeaconPusherAtPowerForSeconds(-1.0, 2.0);

        /*encoderStrafe(DRIVE_SPEED, -3, -3, 5.0);*/

        encoderDrive(TURN_SPEED,   8, -8, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 19,19, 4.0);  // S3: Forward 24 Inches with 4 Sec timeout

        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    public void runBeaconPusherAtPowerForSeconds ( double power, double duration) throws InterruptedException {

        if (opModeIsActive()) {

            ElapsedTime holdTimer = new ElapsedTime();

            // keep looping while we have time remaining.
            robot.beaconPusher.setPosition(power);
            holdTimer.reset();
            while (opModeIsActive() && (holdTimer.time() < duration)) {
                // Update telemetry & Allow time for other processes to run.
                telemetry.update();
                idle();
            }
            robot.beaconPusher.setPosition(0.5);
        }
    }

    public void runBallTransferAtPowerForSeconds (double power, double duration) throws InterruptedException {

        if (opModeIsActive()) {

            ElapsedTime holdTimer = new ElapsedTime();

            // keep looping while we have time remaining.
            robot.ballPickup.setPower(power);
            holdTimer.reset();
            while (opModeIsActive() && (holdTimer.time() < duration)) {
                // Update telemetry & Allow time for other processes to run.
                telemetry.update();
                idle();
            }
            robot.ballTransfer.setPower(0.0);
        }
    }

    public void spinUpBallShooterToPowerForSeconds (double power, double duration) throws InterruptedException {

        if (opModeIsActive()) {

            ElapsedTime holdTimer = new ElapsedTime();

            // keep looping while we have time remaining.
            robot.ballShooter.setPower(power);
            holdTimer.reset();
            while (opModeIsActive() && (holdTimer.time() < duration)) {
                // Update telemetry & Allow time for other processes to run.
                telemetry.update();
                idle();
            }
        }
    }


    public void encoderStrafe(double speed, double leftInches, double rightInches, double timeoutS) throws InterruptedException {
    int newLeftFrontTarget;
    int newRightFrontTarget;
    int newLeftBackTarget;
    int newRightBackTarget;

    double leftFrontInches = leftInches;
    double leftBackInches = leftInches;
    double rightFrontInches = rightInches;
    double rightBackInches = rightInches;

// Ensure that the opmode is still active
if (opModeIsActive()) {

        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = robot.leftFrontWheel.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
        newLeftBackTarget = robot.leftBackWheel.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
        newRightFrontTarget = robot.rightFrontWheel.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
        newRightBackTarget = robot.rightBackWheel.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

        robot.leftFrontWheel.setTargetPosition(newLeftFrontTarget);
        robot.leftBackWheel.setTargetPosition(newLeftBackTarget);
        robot.rightFrontWheel.setTargetPosition(newRightFrontTarget);
        robot.rightBackWheel.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        robot.setWheelMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        robot.strafeLeft(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (opModeIsActive() &&
        (runtime.seconds() < timeoutS) &&
        (robot.leftFrontWheel.isBusy() && robot.rightBackWheel.isBusy() && robot.rightBackWheel.isBusy() && robot.rightFrontWheel.isBusy())) {

        // Display it for the driver.
        telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
        telemetry.addData("Path2",  "Running at %7d :%7d",
        robot.leftFrontWheel.getCurrentPosition(),
        robot.rightFrontWheel.getCurrentPosition());
        telemetry.update();

        // Allow time for other processes to run.
        idle();
        }

        // Stop all motion;
        robot.stopWheels();

        // Turn off RUN_TO_POSITION
        robot.setWheelMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);   // optional pause after each move
        }
        }



    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        double leftFrontInches = leftInches;
        double leftBackInches = leftInches;
        double rightFrontInches = rightInches;
        double rightBackInches = rightInches;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontWheel.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackWheel.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontWheel.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackWheel.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

            robot.leftFrontWheel.setTargetPosition(newLeftFrontTarget);
            robot.leftBackWheel.setTargetPosition(newLeftBackTarget);
            robot.rightFrontWheel.setTargetPosition(newRightFrontTarget);
            robot.rightBackWheel.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.setWheelMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.setAllWheelsToPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftFrontWheel.isBusy() && robot.rightBackWheel.isBusy() && robot.rightBackWheel.isBusy() && robot.rightFrontWheel.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.leftFrontWheel.getCurrentPosition(),
                                            robot.rightFrontWheel.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.stopWheels();

            // Turn off RUN_TO_POSITION
            robot.setWheelMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
}

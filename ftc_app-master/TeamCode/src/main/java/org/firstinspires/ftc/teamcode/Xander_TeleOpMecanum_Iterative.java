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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This file provides basic TeleOp driving for the Innovators 3311 robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common InnovatorBot hardware class to define the devices on the robot.
 * All device access is managed through the HardwareInnovatorBot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 *
 */

@TeleOp(name="Xander_TeleOpMecanum_Iterative", group="Pushbot")
//@Disabled ``````````````````````````````````````````````````````````````                                                              ```````````````````````````````````````````````````````````````
public class Xander_TeleOpMecanum_Iterative extends OpMode{

    /* Declare OpMode members. */
    HardwareInnovatorBot robot       = new HardwareInnovatorBot(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.
    double          servoOffset  = 0.0 ;                  // Servo mid position
    final double    SERVO_SPEED  = 0.02 ;                 // sets rate to move servo


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
        updateTelemetry(telemetry);
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
        double left; // variable to hold value from gamepad1.left_joystick
        double right; // variable to hold value from gamepad1.right_joystick_y
        boolean strafeLeft; // variable to hold value from gamepad1.left_bumper;
        boolean strafeRight; // variable to hold value from gamepad1.right_bumper;
        boolean pickupForward; // variable to hold value from gamepad2.y;
        boolean pickupBackward; // variable to hold value from gamepad2.a;
        boolean servo_up; // variable to hold value value from gamepad1.dpad_up;
        boolean servo_down; // variable to hold value from gamepad1.dpad_down;
        float pickupForward_float; // variable to hold value from gamepad1.right_trigger;
        float pickupBackward_float; // variable to hold value from gamepad1.left_trigger;
        float power = 1;
        // GAMEPAD JOYSTICKS - LEFT AND RIGHT JOYSTICK INDICATES DIRECTION TO DRIVE
        left = -gamepad1.left_stick_y; // double left = value from gamepad1 left joystick UP;
        right = -gamepad1.right_stick_y; // double right = value from gamepad1 right joystick UP;

        // GAMEPAD BUMPERS - DEPRESSED INDICATES STRAFING LEFT OR RIGHT
        // RIGHT BUMPER - Strafe right
        // LEFT BUMPER - Strafe left
        strafeLeft = gamepad1.left_bumper; // boolean strafeLeft = value from gamepad1 left bumper
        strafeRight = gamepad1.right_bumper; // boolean strafeRight = value from gamepad1 right bumper
        if ((!strafeLeft) && (!strafeRight)) { // NO STRAFING, DRIVE WITH JOYSTICKS
            robot.drive(left, right);
        } else {
            if (strafeLeft) { // STRAFE LEFT
                robot.strafeLeft(1);
            } else { // STRAFE RIGHT
                robot.strafeRight(1);
            }
        }

        // GAMEPAD TRIGGERS - RUN BALL PICKUP MOTOR
        // RIGHT TRIGGER - Run ball pickup motor FORWARD to collect balls
        // LEFT TRIGGER - Run ball pickup motor BACKWARD to reject balls
        pickupForward = gamepad2.y; // boolean pickupForward = gamepad2 y
        pickupForward_float = gamepad1.right_trigger; // float pickupForward = gamepad1 right_trigger
        if ((pickupForward) || (pickupForward_float >0)){
            robot.pickupMotor.setPower(1);
        } else {
            pickupBackward = gamepad2.a; // boolean pickupForward = gamepad2 a
            pickupBackward_float = gamepad1.left_trigger; // float pickupForward = gamepad1 left_trigger
            if ((pickupBackward) || (pickupBackward_float >0)) {
                robot.pickupMotor.setPower(-1);
            } else {
                robot.pickupMotor.setPower(0);
            }
        }

        if (gamepad1.dpad_up) {
            robot.driveForwardAtPowerForDuration(1.0, 1.0);
        }
        if (gamepad1.dpad_down) {
            robot.driveForwardAtPowerForDuration(-1.0, 1.0);
        }

        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        updateTelemetry(telemetry);

        telemetry.addData("Raw",    robot.odsSensor.getRawLightDetected());
        telemetry.addData("Normal", robot.odsSensor.getLightDetected());
        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}

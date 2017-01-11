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
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides testing of all of the motors attached to the Innovators 3311 robot.
 * The code is structured as an Iterative OpMode
 */

@TeleOp(name="ZMOTOR_TEST", group="InnoBot")
//@Disabled
public class ZMOTOR_TEST extends OpMode{

    private static final double CSERVO_STOP = 0.5;
    /* Declare OpMode members. */
    InnoBot robot       = new InnoBot_Basic(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.

    /* Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP */
    public void loop() {
        if (gamepad1.dpad_up) {
            robot.rightFrontWheel.setPower(1.0);
        } else if (gamepad1.dpad_down) {
            robot.rightBackWheel.setPower(1.0);
        } else if (gamepad1.dpad_left) {
            robot.leftFrontWheel.setPower(1.0);
        } else if (gamepad1.dpad_right) {
            robot.leftBackWheel.setPower(1.0);
        } else if (gamepad1.left_bumper) {
            robot.ballPickup.setPower(1.0);
        } else if (gamepad1.right_bumper) {
            robot.ballPickup.setPower(-1.0);
        } else if (gamepad1.left_trigger > 0.0) {
            robot.ballTransfer.setPower(1.0);
        } else if (gamepad1.right_trigger > 0.0) {
            robot.ballTransfer.setPower(-1.0);
        } else if (gamepad1.a) {
            robot.ballShooter.setPower(1.0);
        } else {
            robot.rightFrontWheel.setPower(0.0);
            robot.rightBackWheel.setPower(0.0);
            robot.leftFrontWheel.setPower(0.0);
            robot.leftBackWheel.setPower(0.0);
            robot.ballPickup.setPower(0.0);
            robot.ballTransfer.setPower(0.0);
            robot.ballShooter.setPower(0.0);
        }
    }

    /* Code to run ONCE when the driver hits INIT */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Innovators 3311 Driver!");    //

    }

    /* Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY */
    @Override
    public void init_loop() {

    }

    /* Code to run ONCE when the driver hits PLAY */
    @Override
    public void start() {

    }

    /* Code to run ONCE after the driver hits STOP */
    @Override
    public void stop() {

    }

}

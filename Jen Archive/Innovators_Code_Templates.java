/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The op mode assumes that the color sensor
 * is configured with a name of "color sensor".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Innovators_Code_Templates", group = "Sensor")
@Disabled
public class Innovators_Code_Templates extends LinearOpMode {

  /* Declare OpMode members. */

  private ElapsedTime runtime = new ElapsedTime();

  static final double     FORWARD_SPEED = 0.1;
  static final double     TURN_SPEED    = 0.08;

  @Override
  public void runOpMode() throws InterruptedException {

    /*************************************************
     * REFERENCES TO SPECIFIC HARDWARE
     *************************************************/
      InnoBot robot   = new InnoBot_Basic();   // Use a InnovatorBot's hardware



    // get a reference to our Servo object.
    Servo servo1;
    servo1 = hardwareMap.servo.get("S1");

    /*************************************************
     * GYRO SENSOR
     * INITIALIZE VARIABLES USED BY GYRO SENSOR
     * Copy and paste this entire block above the loop in any file using the gyroSensor
     *************************************************/
    // get a reference to a Modern Robotics GyroSensor object.
    ModernRoboticsI2cGyro gyroSensor;   // Hardware Device Object
    gyroSensor = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("GS");
    int iGyroSensor_x, iGyroSensor_y, iGyroSensor_z = 0;     // Gyro rate Values
    int iGyroSensor_heading = 0;                             // Gyro integrated heading
    int iGyroSensor_angleZ = 0;
    boolean bGyroSensor_lastResetState = false;
    boolean bGyroSensor_curResetState  = false;

    // start calibrating the gyro.
    telemetry.addData(">", "Gyro Calibrating. Do Not move!");
    telemetry.update();
    gyroSensor.calibrate();

    // make sure the gyro is calibrated.
    while (gyroSensor.isCalibrating())  {
      Thread.sleep(50);
      idle();
    }

    telemetry.addData(">", "Gyro Calibrated.  Press Start.");
    telemetry.update();

    /*************************************************
     * COLOR SENSOR
     * INITIALIZE VARIABLES USED BY COLOR SENSOR
     * Copy and paste this entire block above the loop in any file using the colorSensor
     *************************************************/
    // get a reference to our ColorSensor object.
    ColorSensor colorSensor;    // Hardware Device Object
    colorSensor = hardwareMap.colorSensor.get("CS");
    // bPrevState and bCurrState represent the previous and current state of the button.
    boolean bColorSensor_LedPrevState = false;
    boolean bColorSensor_LedCurrState = false;

    // bLedOn represents the state of the LED.
    boolean bColorSensor_LedOn = true;

    // Set the LED in the beginning
    colorSensor.enableLed(bColorSensor_LedOn);


    /*************************************************
     * PROGRAM WAIT TO START
     *************************************************/
    // wait for the start button to be pressed.
    waitForStart();


    /*************************************************
     * PROGRAM START - LOOP UNTIL STOP BUTTON ON DRIVER PHONE PUSHED
     *************************************************/
    // Note we use opModeIsActive() as our loop condition because it is an interruptable method.
    while (opModeIsActive()) {


        /*************************************************
         * DRIVING CODE - all of these are options to control the robot movement
         * remove from within comment and add meaningful variable values
         *************************************************/

    /*
        robot.driveForwardAtPowerForDuration(double powerToAllWheelMotors, double durationInSeconds);
        robot.strafeLeft(double powertoAllWheelMotors);
        robot.strafeRight(double powerToAllWheelMotors);
        robot.drive(double powerToLeftWheelMotors, double powerToRightWheelMotors);
        robot.driveBackwardAtPower(double powerToAllWheelMotors);
        robot.driveForwardAtPower(double powerToAllWheelMotors);
        robot.spinCW(double powerToAllWheelMotors);
        robot.spinCCW(double powerToAllWheelMotors);
        robot.spinCCWAtPowerForDuration(double powerToAllWheelMotors, double durationInSeconds);

        */



      /*************************************************
       * COLOR SENSOR CODE
       *************************************************/
      // ColorSensor_Led check the status of the x button on either gamepad.
      bColorSensor_LedCurrState = gamepad1.x;

      // ColorSensor_Led check for button state transitions.
      if ((bColorSensor_LedCurrState == true) && (bColorSensor_LedCurrState != bColorSensor_LedPrevState))  {

        // ColorSensor_Led button is transitioning to a pressed state. So Toggle LED
        bColorSensor_LedOn = !bColorSensor_LedOn;
        colorSensor.enableLed(bColorSensor_LedOn);
      }
      // ColorSensor_Led update previous state variable.
      bColorSensor_LedPrevState = bColorSensor_LedCurrState;

      // ColorSensor_Led send the info back to driver station using telemetry function.
      telemetry.addData("LED", bColorSensor_LedOn ? "On" : "Off");
      telemetry.addData("Clear", colorSensor.alpha());
      telemetry.addData("Red  ", colorSensor.red());
      telemetry.addData("Green", colorSensor.green());
      telemetry.addData("Blue ", colorSensor.blue());

      // ColorSensor_Led test for red or blue
        if (colorSensor.red()>10)
        {
            servo1.setPosition(1);
        } else if (colorSensor.blue()>10)
        {
            servo1.setPosition(0);
        } else {
        }



      /*************************************************
       * GYRO SENSOR CODE
       *************************************************/
      bGyroSensor_curResetState = (gamepad1.a && gamepad1.b);
      if(bGyroSensor_curResetState && !bGyroSensor_lastResetState)  {
        gyroSensor.resetZAxisIntegrator();
      }
      bGyroSensor_lastResetState = bGyroSensor_curResetState;

      // get the x, y, and z values (rate of change of angle).
      iGyroSensor_x = gyroSensor.rawX();
      iGyroSensor_y = gyroSensor.rawY();
      iGyroSensor_z = gyroSensor.rawZ();

      // get the heading info.
      // the Modern Robotics' gyro sensor keeps
      // track of the current heading for the Z axis only.
      iGyroSensor_heading = gyroSensor.getHeading();
      iGyroSensor_angleZ  = gyroSensor.getIntegratedZValue();

      telemetry.addData(">", "Press A & B to reset Heading.");
      telemetry.addData("0", "Heading %03d", iGyroSensor_heading);
      telemetry.addData("1", "Int. Ang. %03d", iGyroSensor_angleZ);
      telemetry.addData("2", "X av. %03d", iGyroSensor_x);
      telemetry.addData("3", "Y av. %03d", iGyroSensor_y);
      telemetry.addData("4", "Z av. %03d", iGyroSensor_z);

      /*************************************************
       * SHOW MESSAGES ON DRIVER PHONE SCREEN
       *************************************************/
      telemetry.update();
      idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
    }
  }
}

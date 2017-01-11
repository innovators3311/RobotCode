package org.firstinspires.ftc.teamcode;

/**
 * Created by Jen on 1/6/2017.
 */

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/************************/
/* driveForwardAtPowerUntilWhiteLineDetected(double power)
/************************/
public  class InnoBot_Basic extends InnoBot {

    public void driveForwardAtPowerUntilWhiteLineDetectedOnFloor(double power, Telemetry telemetry) {
        telemetry.addData("driveForwardAtPowerUntilWhiteLineDetected:", power);
        telemetry.update();

        colorSensor_floor.enableLed(false);
        colorSensor_floor.enableLed(true);

    }

    public void runBallShooterMotorIfBlueBeaconDetected(Telemetry telemetry) {



    }

    public void runBeaconPusherServoIfRedBeaconDetected(Telemetry telemetry) {

        double colorValue_blue;
        double colorValue_red;

        colorSensor_beacon.enableLed(true);

        /*
        // send the info back to driver station using telemetry function.
        telemetry.addData("Clear", colorSensor_beacon.alpha());
        telemetry.addData("Red  ", colorSensor_beacon.red());
        telemetry.addData("Green", colorSensor_beacon.green());
        telemetry.addData("Blue ", colorSensor_beacon.blue());

        colorValue_blue = ((colorSensor_beacon.red() - 0) ^ 2 + (colorSensor_beacon.blue() - 255) ^ 2 + (colorSensor_beacon.green() - 0) ^ 2);
        colorValue_red = ((colorSensor_beacon.red() - 255) ^ 2 + (colorSensor_beacon.blue() - 0) ^ 2 + (colorSensor_beacon.green() - 0) ^ 2);

        if ((colorSensor_beacon.blue() > 2) && (colorSensor_beacon.red() < 2)) {
            telemetry.addData("BEACON", "BLUE");
        } else if ((colorSensor_beacon.blue() < 2) && (colorSensor_beacon.red() > 2)) {
            telemetry.addData("BEACON", "RED");
        } else {
            telemetry.addData("BEACON", "CLEAR");
        }
        telemetry.update();
        */
    }


    public void runBeaconPusherServoIfBlueBeaconDetected(Telemetry telemetry) {

        // colorSensor_beacon
        // beaconPusher

    }











    public void rotateToFaceAngle(double targetAngle, Telemetry telemetry) {
        telemetry.addData("rotateToFaceAngle:", targetAngle);
        telemetry.update();

        double currentHeading;
        double targetHeading;

        boolean goalReached = false;
        float TOLERANCE = 5;

        // start calibrating the gyro.

        currentHeading = gyroSensor.getHeading();
        telemetry.addData("Starting Program - Heading:", currentHeading);
        telemetry.update();

        currentHeading = gyroSensor.getHeading();
        targetHeading = 90;
        telemetry.addData("About to Start Loop - Heading:", currentHeading);
        telemetry.update();
        telemetry.addData("About to Start Loop - Target:", targetHeading);
        telemetry.update();

        while (!goalReached) {
            colorSensor_floor.enableLed(true);
            if (!gyroSensor.isCalibrating()) {
                currentHeading = gyroSensor.getHeading();
                if (Math.abs(currentHeading - targetHeading) < TOLERANCE) {
                    goalReached = true;
                    colorSensor_floor.enableLed(false);
                }
                if (goalReached) {
                    telemetry.addData("In Loop - Goal Reached - Heading:", currentHeading);
                    telemetry.update();
                    stopDriving();
                } else {
                    currentHeading = gyroSensor.getHeading();
                    telemetry.addData("In Loop - Goal Not Reached - Heading:", currentHeading);
                    telemetry.update();
                    telemetry.addData("In Loop - Color Sensor:", colorSensor_floor.red());
                    telemetry.update();
                    spinCW();
                }
            }
        }
    }


    public void rotateAngleDegrees(double angleToRotate, Telemetry telemetry) {};

} // end class InnoBot_Basic

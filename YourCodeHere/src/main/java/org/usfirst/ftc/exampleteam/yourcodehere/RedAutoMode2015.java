package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Autonomous;

import static com.qualcomm.robotcore.hardware.DcMotor.Direction.REVERSE;
import static com.qualcomm.robotcore.hardware.DcMotorController.RunMode.RESET_ENCODERS;
import static com.qualcomm.robotcore.hardware.DcMotorController.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotorController.RunMode.RUN_USING_ENCODERS;
import static com.qualcomm.robotcore.hardware.DcMotorController.RunMode.RUN_WITHOUT_ENCODERS;


@Autonomous(name = "BlueAutoMode2015")
public class RedAutoMode2015 extends SynchronousOpMode {
    // Declare Motors
    DcMotor motorDriveLeft = null;
    DcMotor motorDriveRight = null;
    DcMotor motorBaseRotation = null;
    DcMotor motorWinch = null;
    DcMotor motorBucketArmLateral = null;
    DcMotor motorBucketArmRotation = null;
    DcMotor motorLiftArmRotation = null;
    DcMotor motorLiftArmLateral = null;

    // Declare Servos
    Servo flapLeft = null;
    Servo flapRight = null;
    Servo brushArmLeft = null;
    Servo brushArmRight = null;
    Servo brush = null;


    GyroSensor gyro;


    OpticalDistanceSensor distanceWall;
    OpticalDistanceSensor distanceLine;

    enum LineDirection {
        LEFT, RIGHT, NULL
    }

    LineDirection lineDirection = LineDirection.NULL;

    @Override
    public void main() throws InterruptedException {
        /* Initialize our hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names you assigned during the robot configuration
         * step you did in the FTC Robot Controller app on the phone.
         */
        // check string names in code against the ones on the app

        // Initialize Motors
        motorDriveLeft = hardwareMap.dcMotor.get("motorDriveLeft");
        motorDriveRight = hardwareMap.dcMotor.get("motorDriveRight");
        motorBaseRotation = hardwareMap.dcMotor.get("motorBaseRotation");
        motorWinch = hardwareMap.dcMotor.get("motorWinch");
        motorBucketArmLateral = hardwareMap.dcMotor.get("motorBucketArmLateral");
        motorBucketArmRotation = hardwareMap.dcMotor.get("motorBucketArmRotation");
        motorLiftArmRotation = hardwareMap.dcMotor.get("motorHangArmRotation");
        motorLiftArmLateral = hardwareMap.dcMotor.get("motorHangArmLateral");

        // Initialize Servos
        flapLeft = hardwareMap.servo.get("flapLeft");
        flapRight = hardwareMap.servo.get("flapRight");
        brushArmLeft = hardwareMap.servo.get("brushArmLeft");
        brushArmRight = hardwareMap.servo.get("brushArmRight");
        brush = hardwareMap.servo.get("brush");
        gyro = hardwareMap.gyroSensor.get("gyro");


        // Set Servo Positions
        flapRaised();
        brushArmRaised();
        brush.setPosition(0.5);

        // Set Motor Channel Modes
        motorDriveLeft.setMode(RUN_USING_ENCODERS);
        motorDriveRight.setMode(RUN_USING_ENCODERS);
        motorBaseRotation.setMode(RUN_USING_ENCODERS);
        motorWinch.setMode(RUN_WITHOUT_ENCODERS);
        motorBucketArmLateral.setMode(RUN_WITHOUT_ENCODERS);
        motorBucketArmRotation.setMode(RUN_USING_ENCODERS);
        motorLiftArmRotation.setMode(RUN_USING_ENCODERS);

        // Reverse motorDriveRight so we don't go in circles
        motorDriveLeft.setDirection(REVERSE);


        // turn on LED of light sensor.
        distanceWall.enableLed(true);
        distanceLine.enableLed(true);


        // calibrate the gyro.
        gyro.calibrate();
        while (gyro.isCalibrating()) {
        }

        // Wait for the game to start
        waitForStart();

        // Give the gyro time to calibrate


        // Go R.O.B.I.T. Go!!!
        driveForwardDistance(2260);
        turnToPosition(315);
        driveForwardDistance(6000);
        turnToPosition(270);
        driveForwardDistance(600);
        finalPosition(270);
        dropClimbers();
        distanceWall.enableLed(false);
        distanceLine.enableLed(false);
    }


    public void stopDriving() {
        motorDriveLeft.setPower(0);
        motorDriveRight.setPower(0);
    }


    public void driveForwardDistance(int distance) throws InterruptedException {
        final int heading = gyro.getHeading();
        //Reset Encoders
        motorDriveLeft.setMode(RESET_ENCODERS);
        motorDriveRight.setMode(RESET_ENCODERS);
        while (motorDriveLeft.getCurrentPosition() < distance && motorDriveRight.getCurrentPosition() < distance) {
            int currentHeading = gyro.getHeading();
            if (currentHeading > (heading + 5) % 360 && currentHeading < (heading + 180) % 360) {
                motorDriveRight.setPower(0.5);
                motorDriveLeft.setPower(-0.5);
            } else if (currentHeading > (heading + 180) % 360 && currentHeading < (heading + 355) % 360) {
                motorDriveLeft.setPower(0.5);
                motorDriveRight.setPower(-0.5);
            } else {
                // do nothing
            }

            // Set drive power
            motorDriveLeft.setPower(1);
            motorDriveRight.setPower(1);


        }
        stopDriving();
        motorDriveLeft.setMode(RUN_USING_ENCODERS);
        motorDriveRight.setMode(RUN_USING_ENCODERS);

    }


    public void turnToPosition(int targetHeading) throws InterruptedException {
        int currentHeading = gyro.getHeading();
        while (targetHeading > currentHeading) {
            motorDriveLeft.setPower(-0.3);
            motorDriveRight.setPower(0.3);
        }
    }

    public void finalPosition(int lastTurn) throws InterruptedException {
        if (distanceLine.getLightDetected() <= 0.5 && distanceLine.getLightDetected() >= 1.0) {
            while (distanceWall.getLightDetected() <= 0.5) {
                motorDriveRight.setPower(0.4);
                motorDriveLeft.setPower(0.4);
            }
            motorDriveLeft.setPower(0);
            motorDriveRight.setPower(0);
        } else {
            motorDriveRight.setPower(-0.3);
            motorDriveRight.setPower(-0.3);
            Thread.sleep(500);
            motorDriveRight.setPower(0);
            motorDriveLeft.setPower(0);
            while (distanceWall.getLightDetected() <= 0.4) {
                if (distanceLine.getLightDetected() <= 0.4) {
                    motorDriveRight.setPower(0.3);
                    motorDriveLeft.setPower(0);
                    lineDirection = LineDirection.LEFT;
                } else {
                    motorDriveLeft.setPower(0.3);
                    motorDriveRight.setPower(0);
                    lineDirection = LineDirection.RIGHT;

                }
            }
            if (lineDirection == LineDirection.LEFT) {
                while (gyro.getHeading() < lastTurn) {
                    motorDriveLeft.setPower(0.3);
                    motorDriveRight.setPower(-0.3);
                }
            } else if (lineDirection == LineDirection.RIGHT) {
                while (gyro.getHeading() > lastTurn) {
                    motorDriveLeft.setPower(-0.3);
                    motorDriveRight.setPower(0.3);
                }
            }

        }
    }


    public void dropClimbers() {
        motorLiftArmLateral.setMode(RESET_ENCODERS);
        motorLiftArmLateral.setMode(RUN_TO_POSITION);
        motorLiftArmLateral.setTargetPosition(700);
    }

    private void flapRaised() {
        flapLeft.setPosition(0.13);
        flapRight.setPosition(0.89);
    }


    private void brushArmRaised() {
        brushArmLeft.setPosition(1);
        brushArmRight.setPosition(0);
    }


}



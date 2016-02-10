package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Autonomous;

import static com.qualcomm.robotcore.hardware.DcMotor.Direction.REVERSE;
import static com.qualcomm.robotcore.hardware.DcMotorController.RunMode.RESET_ENCODERS;
import static com.qualcomm.robotcore.hardware.DcMotorController.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotorController.RunMode.RUN_USING_ENCODERS;
import static com.qualcomm.robotcore.hardware.DcMotorController.RunMode.RUN_WITHOUT_ENCODERS;


@Autonomous(name = "RedAutoMode2015")
public class RedAutoMode2015 extends SynchronousOpMode {
    // Declare Motors
    DcMotor motorDriveLeft = null;
    DcMotor motorDriveRight = null;
    DcMotor motorBaseRotation = null;
    DcMotor motorWinch = null;
    DcMotor motorArmLateral = null;
    DcMotor motorArmRotation = null;

    // Declare Servos
    Servo flapLeft = null;
    Servo flapRight = null;
    Servo brushArmLeft = null;
    Servo brushArmRight = null;


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
        motorArmLateral = hardwareMap.dcMotor.get("motorArmLateral");
        motorArmRotation = hardwareMap.dcMotor.get("motorArmRotation");


        //Initialize Servos
        flapLeft = hardwareMap.servo.get("flapLeft");
        flapRight = hardwareMap.servo.get("flapRight");
        brushArmLeft = hardwareMap.servo.get("brushArmLeft");
        brushArmRight = hardwareMap.servo.get("brushArmRight");

        //Set Servo Positions
        flapRaised();
        brushArmRaised();

        // Set Motor Channel Modes
        motorDriveLeft.setMode(RUN_USING_ENCODERS);
        motorDriveRight.setMode(RUN_USING_ENCODERS);
        motorBaseRotation.setMode(RUN_USING_ENCODERS);
        motorWinch.setMode(RUN_WITHOUT_ENCODERS);
        motorArmLateral.setMode(RUN_WITHOUT_ENCODERS);
        motorArmRotation.setMode(RUN_USING_ENCODERS);

        // Reverse motorDriveRight
        motorDriveRight.setDirection(REVERSE);


        // Wait for the game to start
        waitForStart();

        // Go R.O.B.I.T. Go!!!
        driveForwardDistance(1, 1220);


    }

    public void driveForwardDistance(double power, int distance) throws InterruptedException {
        //Reset Encoders
        motorDriveLeft.setMode(RESET_ENCODERS);
        motorDriveRight.setMode(RESET_ENCODERS);

        //Set Target Position
        motorDriveLeft.setTargetPosition(distance);
        motorDriveRight.setTargetPosition(distance);

        // Set ti RUN_TO_POSITION mode
        motorDriveLeft.setMode(RUN_TO_POSITION);
        motorDriveRight.setMode(RUN_TO_POSITION);

        // Set drive power
        motorDriveLeft.setPower(power);
        motorDriveRight.setPower(power);

        while (motorDriveLeft.isBusy() && motorDriveRight.isBusy()) {

        }
        stopDriving();
    }

    public void stopDriving() {
        motorDriveLeft.setPower(0);
        motorDriveRight.setPower(0);
    }


    public void turnLeftDistance(double power, int distance) throws InterruptedException {
        //Reset Encoders
        motorDriveLeft.setMode(RESET_ENCODERS);
        motorDriveRight.setMode(RESET_ENCODERS);

        //Set Target Position
        motorDriveLeft.setTargetPosition(-distance);
        motorDriveRight.setTargetPosition(distance);

        // Set ti RUN_TO_POSITION mode
        motorDriveLeft.setMode(RUN_TO_POSITION);
        motorDriveRight.setMode(RUN_TO_POSITION);

        // Set drive power
        motorDriveLeft.setPower(power);
        motorDriveRight.setPower(power);

        while (motorDriveLeft.isBusy() && motorDriveRight.isBusy()) {

        }
        stopDriving();
    }


    public void turnRightDistance(double power, int distance) throws InterruptedException {
        //Reset Encoders
        motorDriveLeft.setMode(RESET_ENCODERS);
        motorDriveRight.setMode(RESET_ENCODERS);

        //Set Target Position
        motorDriveLeft.setTargetPosition(distance);
        motorDriveRight.setTargetPosition(-distance);

        // Set ti RUN_TO_POSITION mode
        motorDriveLeft.setMode(RUN_TO_POSITION);
        motorDriveRight.setMode(RUN_TO_POSITION);

        // Set drive power
        motorDriveLeft.setPower(power);
        motorDriveRight.setPower(power);

        while (motorDriveLeft.isBusy() && motorDriveRight.isBusy()) {

        }
        stopDriving();
    }

    public void driveBackwardsDistance(double power, int distance) throws InterruptedException {
        //Reset Encoders
        motorDriveLeft.setMode(RESET_ENCODERS);
        motorDriveRight.setMode(RESET_ENCODERS);

        //Set Target Position
        motorDriveLeft.setTargetPosition(-distance);
        motorDriveRight.setTargetPosition(-distance);

        // Set ti RUN_TO_POSITION mode
        motorDriveLeft.setMode(RUN_TO_POSITION);
        motorDriveRight.setMode(RUN_TO_POSITION);

        // Set drive power
        motorDriveLeft.setPower(power);
        motorDriveRight.setPower(power);

        while (motorDriveLeft.isBusy() && motorDriveRight.isBusy()) {

        }
        stopDriving();
    }

    public void dumpDebris() throws InterruptedException {
        motorArmRotation.setPower(1);
        wait(400);
        motorArmLateral.setPower(1);
        wait(300);
        brushArmRaised();
        flapLoweredTime();
        wait(1000);
        flapRaised();
        wait(100);
        motorArmLateral.setPower(-1);
        wait(300);
        motorArmRotation.setPower(-1);
        wait(400);
    }

    public void turnBase180() throws InterruptedException {
        motorBaseRotation.setPower(1);
        wait(1500);
    }

    public void flapRaised() {
        flapLeft.setPosition(0);
        flapRight.setPosition(1);
    }

    public void flapLowered() {
        flapLeft.setPosition(0.5);
        flapRight.setPosition(0.5);
    }

    private void brushArmRaised() {
        brushArmLeft.setPosition(1);
        brushArmRight.setPosition(0);
    }

    private void flapLoweredTime() throws InterruptedException {
        while (flapLeft.getPosition() < 0.72) {
            flapLeft.setPosition(flapLeft.getPosition() + 0.03);
            flapRight.setPosition(flapRight.getPosition() - 0.03);
            Thread.sleep(100);
        }
    }
}


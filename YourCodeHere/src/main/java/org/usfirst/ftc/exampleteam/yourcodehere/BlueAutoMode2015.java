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


@Autonomous(name = "BlueAutoMode2015")
public class BlueAutoMode2015 extends SynchronousOpMode {
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
    Servo brush = null;


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
        brush = hardwareMap.servo.get("brush");

        //Set Servo Positions
        flapRaised();
        brushArmRaised();
        brush.setPosition(0.5);

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
        startUP();
        driveForwardDistance(9500);
        turnLeftDistance(2500);
        driveForwardDistance(4500);
        //dumpDebris();
    }

    public void stopDriving() {
        motorDriveLeft.setPower(0);
        motorDriveRight.setPower(0);
    }


    public void driveForwardDistance(int distance) throws InterruptedException {
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
        motorDriveLeft.setPower(1);
        motorDriveRight.setPower(1);

        while (motorDriveLeft.getCurrentPosition() > -distance && motorDriveRight.getCurrentPosition() > -distance) {

        }
        stopDriving();
        motorDriveLeft.setMode(RUN_USING_ENCODERS);
        motorDriveRight.setMode(RUN_USING_ENCODERS);
    }

    public void turnLeftDistance(int distance) throws InterruptedException {
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
        motorDriveLeft.setPower(1);
        motorDriveRight.setPower(1);

        while (motorDriveLeft.getCurrentPosition() > -distance && motorDriveRight.getCurrentPosition() < distance) {

        }
        stopDriving();
        motorDriveLeft.setMode(RUN_USING_ENCODERS);
        motorDriveRight.setMode(RUN_USING_ENCODERS);
    }


    public void turnRightDistance(int distance) throws InterruptedException {
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
        motorDriveLeft.setPower(1);
        motorDriveRight.setPower(1);

        while (motorDriveLeft.getCurrentPosition() < distance && motorDriveRight.getCurrentPosition() > -distance) {

        }
        stopDriving();
        motorDriveLeft.setMode(RUN_USING_ENCODERS);
        motorDriveRight.setMode(RUN_USING_ENCODERS);
    }

    public void driveBackwardsDistance(int distance) throws InterruptedException {
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
        motorDriveLeft.setPower(1);
        motorDriveRight.setPower(1);

        while (motorDriveLeft.getCurrentPosition() < distance && motorDriveRight.getCurrentPosition() < distance) {

        }
        stopDriving();
        motorDriveLeft.setMode(RUN_USING_ENCODERS);
        motorDriveRight.setMode(RUN_USING_ENCODERS);
    }

    public void dumpDebris() throws InterruptedException {
        motorArmRotation.setMode(RUN_WITHOUT_ENCODERS);
        motorArmLateral.setPower(1);
        Thread.sleep(1500);
        motorArmLateral.setPower(0);
        motorArmRotation.setPower(-0.5);
        Thread.sleep(1000);
        holdArm();
        motorArmLateral.setPower(1);
        Thread.sleep(2000);
        motorArmLateral.setPower(0);
        brushArmRaised();
        flapLoweredTime();
        motorArmLateral.setPower(-1);
        Thread.sleep(2000);
        motorArmLateral.setPower(0);
        motorArmRotation.setPower(0.1);
        Thread.sleep(3000);
        motorArmRotation.setPower(0);
    }


    private void flapRaised() {
        flapLeft.setPosition(0.13);
        flapRight.setPosition(0.89);
    }

    private void flapLowered() {
        flapLeft.setPosition(0.72);
        flapRight.setPosition(0.35);
    }

    private void brushArmRaised() {
        brushArmLeft.setPosition(1);
        brushArmRight.setPosition(0);
    }

    private void brushArmLowered() {
        brushArmLeft.setPosition(0);
        brushArmRight.setPosition(1);
    }

    private void flapLoweredTime() throws InterruptedException {
        while (flapLeft.getPosition() < 0.72) {
            flapLeft.setPosition(flapLeft.getPosition() + 0.03);
            flapRight.setPosition(flapRight.getPosition() - 0.03);
            Thread.sleep(100);
        }
    }

    private void holdArm() {
        motorArmRotation.setTargetPosition(motorArmRotation.getCurrentPosition());
        motorArmRotation.setMode(RUN_TO_POSITION);
        motorArmRotation.setPower(1);
    }

    private void holdBase() {
        motorBaseRotation.setTargetPosition(motorBaseRotation.getCurrentPosition());
        motorBaseRotation.setMode(RUN_TO_POSITION);
        motorBaseRotation.setPower(1);
    }

    private void startUP() throws InterruptedException {
        brushArmLowered();
        brush.setPosition(1);
        holdBase();
        motorArmLateral.setPower(1);
        Thread.sleep(900);
        motorArmLateral.setPower(0);
        motorArmRotation.setPower(-0.6);
        Thread.sleep(600);
        motorArmRotation.setPower(0);
        holdArm();
    }
}



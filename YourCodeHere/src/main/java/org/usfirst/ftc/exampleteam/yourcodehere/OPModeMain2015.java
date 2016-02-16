package org.usfirst.ftc.exampleteam.yourcodehere;

//import android.hardware.Camera;
//import android.hardware.Camera.Parameters;

import com.firebase.client.Firebase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.TeleOp;

import static com.qualcomm.robotcore.hardware.DcMotorController.RunMode.RESET_ENCODERS;
import static com.qualcomm.robotcore.hardware.DcMotorController.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotorController.RunMode.RUN_USING_ENCODERS;
import static com.qualcomm.robotcore.hardware.DcMotorController.RunMode.RUN_WITHOUT_ENCODERS;


@TeleOp(name = "OPModeMain2015")
public class OPModeMain2015 extends SynchronousOpMode {

    enum BrushRunState {
        RUNNING, STOPPED
    }

    enum BrushDirection {
        FORWARD, BACKWARD
    }

    enum ArmDirection {
        UP, DOWN
    }

    BrushRunState brushRunState = BrushRunState.STOPPED;
    BrushDirection brushDirection = BrushDirection.FORWARD;
    ArmDirection armDirection = ArmDirection.UP;

    // Declare Motors
    DcMotor motorDriveLeft = null;
    DcMotor motorDriveRight = null;
    DcMotor motorBaseRotation = null;
    DcMotor motorArmLateral = null;
    DcMotor motorArmRotation = null;

    // Declare Servos
    Servo brush = null;
    Servo flapLeft = null;
    Servo flapRight = null;
    Servo brushArmLeft = null;
    Servo brushArmRight = null;
    Servo leftClimber = null;
    Servo rightClimber = null;

    double armHookPosition = -1350;
    int armPosition;
    int armIncrement = 15;
    boolean startUP = false;
    boolean armsetpos = true;


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
        motorArmLateral = hardwareMap.dcMotor.get("motorArmLateral");
        motorArmRotation = hardwareMap.dcMotor.get("motorArmRotation");


        //Initialize Servos
        brush = hardwareMap.servo.get("brush");
        flapLeft = hardwareMap.servo.get("flapLeft");
        flapRight = hardwareMap.servo.get("flapRight");
        brushArmLeft = hardwareMap.servo.get("brushArmLeft");
        brushArmRight = hardwareMap.servo.get("brushArmRight");
        leftClimber = hardwareMap.servo.get("leftClimber");
        rightClimber = hardwareMap.servo.get("rightClimber");

        //Set Servo Positions
        brush.setPosition(0.5);
        flapRaised();
        brushArmRaised();
        climbersin();

        // Set Motor Channel Modes
        motorDriveLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorDriveRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorBaseRotation.setMode(RUN_USING_ENCODERS);
        motorArmLateral.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorArmRotation.setMode(RESET_ENCODERS);
        motorArmRotation.setMode(RUN_USING_ENCODERS);

        // Reverse motorDriveRight
        motorDriveRight.setDirection(DcMotor.Direction.REVERSE);

        //Initialize Firebase

        Firebase firebaseRef = new Firebase("https://9523-2015.firebaseio.com/");

        // Wait for the game to start
        waitForStart();

        // Go R.O.B.I.T. Go!!!
        while (opModeIsActive()) {
            if (updateGamepads()) {
                if (startUP == false) {
                    brushArmLowered();
                    brush.setPosition(1);
                    startUP = true;
                }

                //======================================================================================================================================================================
                // Arcade Drive
                //get the values from the gamepads
                //note: pushing the stick all the way up returns -1, so we need to reverse the y values
                float xValue = gamepad1.right_stick_x;
                float yValue = -gamepad1.right_stick_y;

                //calculate the power needed for each motor
                float leftPower = yValue + xValue;
                float rightPower = yValue - xValue;

                //clip the power values so that it only goes from -1 to 1
                leftPower = Range.clip(leftPower, -1, 1);
                rightPower = Range.clip(rightPower, -1, 1);

                //set the power of the motors with the gamepad values
                motorDriveLeft.setPower(leftPower);
                motorDriveRight.setPower(rightPower);


                //======================================================================================================================================================================
                // Base Rotation

                if (gamepad2.right_stick_x == 1.0) {
                    motorBaseRotation.setMode(RUN_USING_ENCODERS);
                    motorBaseRotation.setPower(-0.2);
                } else if (gamepad2.right_stick_x == -1.0) {
                    motorBaseRotation.setMode(RUN_USING_ENCODERS);
                    motorBaseRotation.setPower(0.2);
                } else {
                    motorBaseRotation.setPower(0);
                }

                /*else if (motorBaseRotation.getMode() != RUN_TO_POSITION) {
                       motorBaseRotation.setTargetPosition(motorBaseRotation.getCurrentPosition());
                        motorBaseRotation.setMode(RUN_TO_POSITION);
                        motorBaseRotation.setPower(0.2);
                }*/

                //======================================================================================================================================================================
                //Arm Rotation
                /*if (gamepad2.a) {
                    while ((motorArmRotation.getCurrentPosition() != armHookPosition)) {
                        armPosition = motorArmRotation.getCurrentPosition();
                        motorArmRotation.setTargetPosition(armPosition - armIncrement);
                        motorArmRotation.setMode(RUN_TO_POSITION);
                        motorArmRotation.setPower(-0.7);
                        while (motorArmRotation.isBusy()) {

                        }
                    }
                }*/


               /* if (gamepad2.y){
                    if(armDirection == ArmDirection.UP){
                        armDirection = ArmDirection.DOWN;
                        telemetry.addData("armDirection", armDirection);
                        telemetry.update();
                    }
                    else{
                        armDirection = ArmDirection.UP;
                        telemetry.addData("armDirection", armDirection);
                        telemetry.update();
                    }
                }*/

                if (gamepad2.right_stick_y == 1.0) {
                    motorArmRotation.setMode(RUN_WITHOUT_ENCODERS);
                    motorArmRotation.setPower(-0.1);
                } else if (gamepad2.right_stick_y == -1.0) {
                    motorArmRotation.setMode(RUN_USING_ENCODERS);
                    motorArmRotation.setPower(-0.6);
                }

                /*else if((gamepad2.right_stick_y == 1.0) && armDirection == ArmDirection.DOWN){
                    motorArmRotation.setMode(RUN_USING_ENCODERS);
                    motorArmRotation.setPower(0.6);
                }

                else if((gamepad2.right_stick_y == -1.0) && armDirection == ArmDirection.DOWN){
                    motorArmRotation.setMode(RUN_WITHOUT_ENCODERS);
                    motorArmRotation.setPower(0.1);
                }*/

                else if (motorArmRotation.getMode() != RUN_TO_POSITION) {
                    motorArmRotation.setTargetPosition(motorArmRotation.getCurrentPosition());
                    motorArmRotation.setMode(RUN_TO_POSITION);
                    motorArmRotation.setPower(1);
                }

                //======================================================================================================================================================================
                // Arm Lateral Movement
                motorArmLateral.setPower(gamepad2.left_stick_y * 0.8);

                //======================================================================================================================================================================
                //Move Brush Arm

                if (gamepad2.dpad_up) {
                    brushArmRaised();
                } else if (gamepad2.dpad_down) {
                    brushArmLowered();
                }

                if (gamepad2.dpad_left) {
                    flapRaised();
                } else if (gamepad2.dpad_right) {
                    flapLowered();
                }


                //======================================================================================================================================================================
                // Spin the brush forward and backward

                if (gamepad2.left_bumper) {
                    brush.setPosition(1);
                    brushRunState = BrushRunState.RUNNING;
                } else if (gamepad2.right_bumper) {
                    brush.setPosition(0.5);
                    brushRunState = BrushRunState.STOPPED;
                }

                if (gamepad2.y && brushRunState == BrushRunState.RUNNING) {
                    if (brushDirection == BrushDirection.FORWARD) {
                        brush.setPosition(1);
                        brushDirection = BrushDirection.BACKWARD;
                    } else {
                        brush.setPosition(0);
                        brushDirection = BrushDirection.FORWARD;
                    }
                }


                //======================================================================================================================================================================
                // Drop the debris
                if (gamepad2.b) {
                    dropDebris();
                }

                //======================================================================================================================================================================
                //Hit Climbers
                if (gamepad1.left_bumper) {
                    hitLeftClimber();
                }

                if (gamepad1.right_bumper) {
                    hitRightClimber();
                }

            }
        }


        telemetry.update();
        idle();
    }

    private void dropDebris() throws InterruptedException {
        brushArmRaised();
        flapLoweredTime();
        Thread.sleep(1000);
        flapRaised();
        brushArmLowered();
    }

    private void brushArmRaised() {
        brushArmLeft.setPosition(1);
        brushArmRight.setPosition(0);
    }

    private void brushArmLowered() {
        brushArmLeft.setPosition(0);
        brushArmRight.setPosition(1);
    }

    private void flapRaised() {
        flapLeft.setPosition(0.03);
        flapRight.setPosition(0.99);
    }

    private void flapLowered() {
        flapLeft.setPosition(1);
        flapRight.setPosition(0.07);
    }

    private void flapLoweredTime() throws InterruptedException {
        while (flapLeft.getPosition() < 0.9) {
            flapLeft.setPosition(flapLeft.getPosition() + 0.03);
            flapRight.setPosition(flapRight.getPosition() - 0.03);
            Thread.sleep(100);
        }
    }


    private void climbersin() {
        leftClimber.setPosition(0);
        rightClimber.setPosition(1);
    }

    private void hitLeftClimber() throws InterruptedException {
        leftClimber.setPosition(1);
        Thread.sleep(1000);
        leftClimber.setPosition(0);
    }

    private void hitRightClimber() throws InterruptedException {
        rightClimber.setPosition(0);
        Thread.sleep(1000);
        rightClimber.setPosition(1);
    }

    /*private double armLinearPower(int armPosition) {
        int val = (int) ((0.7 * (1200 - armPosition) / 1200) * 100);
        return val / 100.0;
    }*/
}



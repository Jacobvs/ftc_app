package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.TeleOp;

import static com.qualcomm.robotcore.hardware.DcMotorController.RunMode.RESET_ENCODERS;
import static com.qualcomm.robotcore.hardware.DcMotorController.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotorController.RunMode.RUN_USING_ENCODERS;

// Need to figure out what needs to be imported for continuous Servo

/*
This is my first tele-op drive code
 */
@TeleOp(name = "ArmRotation")
public class ArmRotation extends SynchronousOpMode {
    // Declare Motors
    DcMotor motorArmRotation = null;

    int armHookPosition = -1400;
    int armPosition;
    int armIncrement = 20;
    boolean bail = false;

    @Override
    public void main() throws InterruptedException {
        /* Initialize our hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names you assigned during the robot configuration
         * step you did in the FTC Robot Controller app on the phone.
         */
        // check string names in code against the ones on the app

        // Initialize Motors
        motorArmRotation = hardwareMap.dcMotor.get("motorArmRotation");

        // Set Motor Channel Modes
        motorArmRotation.setMode(RESET_ENCODERS);
        motorArmRotation.setMode(RUN_USING_ENCODERS);

        // Wait for the game to start
        waitForStart();

        long lastUpdate = 0;
        // Go robot!
        while (opModeIsActive()) {
            if (updateGamepads()) {
                // Test Arcade Drive
                //get the values from the gamepads
                //note: pushing the stick all the way up returns -1, so we need to reverse the y values
            /*if(System.currentTimeMillis()-lastUpdate > 5) {
                if (gamepad1.right_stick_y == 0.00) {
                    motorArmRotation.setMode(RESET_ENCODERS);
                    motorArmRotation.setTargetPosition(motorArmRotation.getCurrentPosition());
                    motorArmRotation.setMode(RUN_TO_POSITION);
                    motorArmRotation.setPower(1);
                }
                if (gamepad1.right_stick_y != 0.00) {
                    motorArmRotation.setMode(RESET_ENCODERS);
                    motorArmRotation.setMode(RUN_USING_ENCODERS);
                    motorArmRotation.setPower(gamepad1.right_stick_y);
                }
                lastUpdate = System.currentTimeMillis();
            }*/
                /*if (gamepad1.right_stick_y != 0.0) {
                    motorArmRotation.setMode(RUN_USING_ENCODERS);
                    motorArmRotation.setPower(Range.clip(gamepad1.right_stick_y, -0.7, 0.7));}
                else if(motorArmRotation.getMode() != RUN_TO_POSITION) {
                    motorArmRotation.setTargetPosition(motorArmRotation.getCurrentPosition());
                    motorArmRotation.setMode(RUN_TO_POSITION);
                    motorArmRotation.setPower(1.0);
                }*/

                if (gamepad2.a) {
                    while ((motorArmRotation.getCurrentPosition() != armHookPosition)) {
                        armPosition = motorArmRotation.getCurrentPosition();
                        motorArmRotation.setTargetPosition(armPosition - armIncrement);
                        motorArmRotation.setMode(RUN_TO_POSITION);
                        motorArmRotation.setPower(-0.7);
                        while (motorArmRotation.isBusy()) {

                        }
                    }
                }

                /*if(gamepad1.a){
                    motorArmRotation.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    motorArmRotation.setTargetPosition(1440);
                    motorArmRotation.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                    motorArmRotation.setPower(0.1);
                    while (motorArmRotation.isBusy()) {

                    }
                    motorArmRotation.setPower(0);
                    motorArmRotation.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                */
                telemetry.update();
                idle();
            }
            telemetry.addData("outofif", "outofif");
            telemetry.update();

            //motorArmRotation.setPower(gamepad2.left_stick_y);

        }
    }

    /*private double armLinearPower(int armPosition) {
        int val = (int) ((0.7 * (1200 - armPosition) / 1200) * 100);
        return ((val / 100.0)*-1.0)-0.2;
    }*/
}
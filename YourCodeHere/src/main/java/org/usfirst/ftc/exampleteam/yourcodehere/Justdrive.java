package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.TeleOp;

//import com.qualcomm.robotcore.hardware.Servo;
// Need to figure out what needs to be imported for continuous Servo

/*
This is my first tele-op drive code
 */
@TeleOp(name = "justDrive")
public class Justdrive extends SynchronousOpMode {
    // Declare Motors
    DcMotor motorDriveLeft = null;
    DcMotor motorDriveRight = null;

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

        // Set Motor Channel Modes
        motorDriveLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorDriveRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        // Wait for the game to start
        waitForStart();

        // Go robot!
        while (opModeIsActive()) {
            if (updateGamepads()) {
                // Test Arcade Drive
                //get the values from the gamepads
                //note: pushing the stick all the way up returns -1, so we need to reverse the y values
                float xValue = gamepad1.left_stick_x;
                float yValue = -gamepad1.left_stick_y;

                //calculate the power needed for each motor
                float leftPower = yValue + xValue;
                float rightPower = yValue - xValue;

                //clip the power values so that it only goes from -1 to 1
                leftPower = Range.clip(leftPower, -1, 1);
                rightPower = Range.clip(rightPower, -1, 1);

                //set the power of the motors with the gamepad values
                motorDriveLeft.setPower(leftPower);
                motorDriveRight.setPower(rightPower);

                telemetry.update();
                idle();
            }
        }
    }
}
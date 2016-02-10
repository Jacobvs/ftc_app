package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.TeleOp;

/**
 * A skeletal example of a do-nothing first OpMode. Go ahead and change this code
 * to suit your needs, or create sibling OpModes adjacent to this one in the same
 * Java package.
 */
@TeleOp(name = "ServoTest")
public class ServoTest extends SynchronousOpMode {
    /* Declare all of the servos  */
    Servo brushArmLeft = null;
    Servo brushArmRight = null;
    Servo brush = null;
    Servo flapLeft = null;
    Servo flapRight = null;


    @Override
    public void main() throws InterruptedException {
        /* Initialize our hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names you assigned during the robot configuration
         * step you did in the FTC OpMode15 Controller app on the phone.
         */
        //Initialize Servos
        brushArmLeft = hardwareMap.servo.get("brushArmLeft");
        brushArmRight = hardwareMap.servo.get("brushArmRight");
        brush = hardwareMap.servo.get("brush");
        flapLeft = hardwareMap.servo.get("flapLeft");
        flapRight = hardwareMap.servo.get("flapRight");


        // Wait for the game to start
        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive()) {
            if (updateGamepads()) {

                if (gamepad1.a) {
                    brushArmRight.setPosition(0.5);
                } else if (gamepad1.b) {
                    flapRight.setPosition(0.5);
                } else if (gamepad1.y) {
                    brushArmLeft.setPosition(0.5);
                } else if (gamepad1.x) {
                    flapLeft.setPosition(0.5);
                }
            }


            telemetry.update();
            idle();
        }
    }
}

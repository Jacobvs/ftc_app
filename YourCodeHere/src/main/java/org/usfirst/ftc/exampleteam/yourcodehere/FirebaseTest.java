package org.usfirst.ftc.exampleteam.yourcodehere;

import com.firebase.client.Firebase;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.TeleOp;

/**
 * A skeletal example of a do-nothing first OpMode. Go ahead and change this code
 * to suit your needs, or create sibling OpModes adjacent to this one in the same
 * Java package.
 */
@TeleOp(name = "My First OpMode")
public class FirebaseTest extends SynchronousOpMode {
    /* Declare here any fields you might find useful. */
    // DcMotor motorLeft = null;
    // DcMotor motorRight = null;

    ColorSensor color;

    @Override
    public void main() throws InterruptedException {
        /* Initialize our hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names you assigned during the robot configuration
         * step you did in the FTC Robot Controller app on the phone.
         */

        //this.color = this.hardwareMap.colorSensor.get("colorSensor");

        Firebase firebaseRef = new Firebase("https://9523-2015.firebaseio.com/");

        // Wait for the game to start
        waitForStart();

        // Go go gadget robot!


        firebaseRef.setValue("hello world");

        Thread.sleep(5000);


    }
}
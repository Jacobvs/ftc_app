package org.usfirst.ftc.exampleteam.yourcodehere;

//import android.hardware.Camera;
//import android.hardware.Camera.Parameters;

import com.firebase.client.Firebase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.TeleOp;

import org.athenian.ftc.ListenerAction;
import org.athenian.ftc.RobotValues;
import org.athenian.ftc.ValueListener;
import org.athenian.ftc.ValueSource;
import org.athenian.ftc.ValueWriter;

import java.util.concurrent.ScheduledThreadPoolExecutor;

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

  ScheduledThreadPoolExecutor executor = new ScheduledThreadPoolExecutor(1);

  RobotValues robotValues = null;

  BrushRunState brushRunState = BrushRunState.STOPPED;
  BrushDirection brushDirection = BrushDirection.FORWARD;

  // Declare Motors
  DcMotor motorDriveLeft = null;
  DcMotor motorDriveRight = null;
  DcMotor motorBaseRotation = null;
  DcMotor motorBucketArmLateral = null;
  DcMotor motorBucketArmRotation = null;
  DcMotor motorHangArmRotation = null;
  DcMotor motorHangArmLateral = null;
  DcMotor motorWinch = null;

  // Declare Servos
  Servo brush = null;
  Servo flapLeft = null;
  Servo flapRight = null;
  Servo brushArmLeft = null;
  Servo brushArmRight = null;

  //Declare Sensors
  GyroSensor gyro;
  OpticalDistanceSensor distanceWall;
  OpticalDistanceSensor distanceLine;


  boolean startUP = false;
  boolean hangArmOut = false;


  @Override
  public void main() throws InterruptedException {
        /* Initialize our hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names you assigned during the robot configuration
         * step you did in the FTC Robot Controller app on the phone.
         */
    // check string names in code against the ones on the app

    FirebaseInit();

    // Initialize Motors
    motorDriveLeft = hardwareMap.dcMotor.get("motorDriveLeft");
    motorDriveRight = hardwareMap.dcMotor.get("motorDriveRight");
    motorBaseRotation = hardwareMap.dcMotor.get("motorBaseRotation");
    motorBucketArmLateral = hardwareMap.dcMotor.get("motorBucketArmLateral");
    motorBucketArmRotation = hardwareMap.dcMotor.get("motorBucketArmRotation");
    motorHangArmRotation = hardwareMap.dcMotor.get("motorHangArmRotation");
    motorHangArmLateral = hardwareMap.dcMotor.get("motorHangArmLateral");
    motorWinch = hardwareMap.dcMotor.get("motorWinch");

    //Initialize Servos
    brush = hardwareMap.servo.get("brush");
    flapLeft = hardwareMap.servo.get("flapLeft");
    flapRight = hardwareMap.servo.get("flapRight");
    brushArmLeft = hardwareMap.servo.get("brushArmLeft");
    brushArmRight = hardwareMap.servo.get("brushArmRight");

    //Initialize Sensors
    gyro = hardwareMap.gyroSensor.get("gyro");
    distanceWall = hardwareMap.opticalDistanceSensor.get("distanceWall");
    distanceLine = hardwareMap.opticalDistanceSensor.get("distanceLine");

    //Set Servo Positions
    brush.setPosition(0.5);
    flapRaised();
    brushArmRaised();

    // Set Motor Channel Modes
    motorDriveLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    motorDriveRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    motorBucketArmLateral.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    motorHangArmLateral.setMode(RUN_WITHOUT_ENCODERS);
    motorBucketArmRotation.setMode(RESET_ENCODERS);
    motorBucketArmRotation.setMode(RUN_USING_ENCODERS);
    motorHangArmRotation.setMode(RESET_ENCODERS);
    motorHangArmRotation.setMode(RUN_USING_ENCODERS);

    // Reverse motorDriveRight
    motorDriveRight.setDirection(DcMotor.Direction.REVERSE);

    //Let Gyro Calibrate
    gyro.calibrate();
    while (gyro.isCalibrating()) {
    }

    // Wait for the game to start
    waitForStart();


    // Go R.O.B.I.T. Go!!!
    while (opModeIsActive()) {
      if (updateGamepads()) {
        if (startUP == false) {
          brushArmLowered();
          brush.setPosition(1);
          //motorHangArmRotation.setTargetPosition(x);
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
          motorBaseRotation.setPower(-0.2);
        } else if (gamepad2.right_stick_x == -1.0) {
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

        if (gamepad2.right_stick_y == 1.0) {
          motorBucketArmRotation.setMode(RUN_WITHOUT_ENCODERS);
          motorBucketArmRotation.setPower(-0.1);
        } else if (gamepad2.right_stick_y == -1.0) {
          motorBucketArmRotation.setMode(RUN_USING_ENCODERS);
          motorBucketArmRotation.setPower(-0.6);
        } else if (motorBucketArmRotation.getMode() != RUN_TO_POSITION) {
          motorBucketArmRotation.setTargetPosition(motorBucketArmRotation.getCurrentPosition());
          motorBucketArmRotation.setMode(RUN_TO_POSITION);
          motorBucketArmRotation.setPower(1);
        }

        //======================================================================================================================================================================
        // Arm Lateral Movement
        motorBucketArmLateral.setPower(gamepad2.left_stick_y * 0.4);

        //======================================================================================================================================================================
        //Winch

        if (gamepad2.dpad_up) {
          motorWinch.setPower(1);
        }
        if (gamepad2.dpad_down) {
          motorWinch.setPower(-1);
        } else {
          motorWinch.setPower(0);
        }

        //======================================================================================================================================================================
        //Hook Arm Rotation

        if (gamepad1.left_stick_y == 1.0) {
          motorBucketArmLateral.setPower(0.4);
        } else if (gamepad1.left_stick_y == -1.0) {
          motorBucketArmRotation.setPower(0.4);
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
        //

        if (gamepad1.a && hangArmOut == false) {
          executor.execute(
              new Runnable() {
                @Override
                public void run() {
                  try {
                    motorHangArmLateral.setPower(0.3);
                    Thread.sleep(1000);
                    motorHangArmLateral.setPower(0);
                  } catch (InterruptedException e) {
                    e.printStackTrace();
                  }
                }
              });
          hangArmOut = true;
        }
        if (gamepad1.y && hangArmOut == true) {
          executor.execute(
              new Runnable() {
                @Override
                public void run() {
                  try {
                    motorHangArmLateral.setPower(0.3);
                    Thread.sleep(100);
                    motorHangArmLateral.setPower(0);
                  } catch (InterruptedException e) {
                    e.printStackTrace();
                  }
                }
              });
        } else if (gamepad1.a && hangArmOut == true) {
          executor.execute(
              new Runnable() {
                @Override
                public void run() {
                  try {
                    motorHangArmLateral.setPower(-0.3);
                    Thread.sleep(100);
                    motorHangArmLateral.setPower(0);
                  } catch (InterruptedException e) {
                    e.printStackTrace();
                  }
                }
              });
        }

        //======================================================================================================================================================================
        // Drop the debris
        if (gamepad2.b) {
          executor.execute(new Runnable() {
            @Override
            public void run() {
              try {
                dropDebris();
              } catch (InterruptedException e) {
                e.printStackTrace();
              }
            }
          });
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

    /*private double armLinearPower(int armPosition) {
        int val = (int) ((0.7 * (1200 - armPosition) / 1200) * 100);
        return val / 100.0;
    }*/


  public void FirebaseInit() {
    final Firebase fb = new Firebase("https://your-firebase-url.firebaseio.com/");

    this.robotValues = new RobotValues(fb, 1.5);
    this.robotValues
        .add(new ValueWriter("Sensor.Gyroscopic.Gyro 1",
            new ValueSource() {
              @Override
              public Object getValue() {
                return String.format("The values are- Heading: %s, Rotation: %s, Raw x: %s, Raw y: %s, Raw z: %s",
                    gyro.getHeading(), gyro.getRotation(), gyro.rawX(), gyro.rawY(), gyro.rawZ());
              }
            }))
        .add(new ValueWriter("Sensor.reflection.Wall",
            new ValueSource() {
              @Override
              public Object getValue() {
                return distanceWall.getLightDetected();
              }
            }))
        .add(new ValueWriter("Sensor.reflection.Line",
            new ValueSource() {
              @Override
              public Object getValue() {
                return distanceLine.getLightDetected();
              }
            }))
        .add(new ValueWriter("Motor.Drive Left",
            new ValueSource() {
              @Override
              public Object getValue() {
                return motorDriveLeft.getCurrentPosition();
              }
            }))
        .add(new ValueWriter("Motor.Drive Right",
            new ValueSource() {
              @Override
              public Object getValue() {
                return motorDriveRight.getCurrentPosition();
              }
            }))
        .add(new ValueWriter("motor.Bucket Arm",
            new ValueSource() {
              @Override
              public Object getValue() {
                return motorBucketArmRotation.getCurrentPosition();
              }
            }))
        .add(new ValueWriter("motor.Hang Arm",
            new ValueSource() {
              @Override
              public Object getValue() {
                return motorHangArmRotation.getCurrentPosition();
              }
            }))
        .add(new ValueWriter("servo.Brush",
            new ValueSource() {
              @Override
              public Object getValue() {
                return brush.getPosition();
              }
            }))
        .add(new ValueWriter("servo.Arm Left",
            new ValueSource() {
              @Override
              public Object getValue() {
                return brushArmLeft.getPosition();
              }
            }))
        .add(new ValueWriter("servo.Arm Right",
            new ValueSource() {
              @Override
              public Object getValue() {
                return brushArmRight.getPosition();
              }
            }))
        .add(new ValueWriter("servo.Flap Left",
            new ValueSource() {
              @Override
              public Object getValue() {
                return flapLeft.getPosition();
              }
            }))
        .add(new ValueWriter("servo.Flap Right",
            new ValueSource() {
              @Override
              public Object getValue() {
                return flapRight.getPosition();
              }
            }))
        .add(new ValueListener("motor.Drive Left",
            new ListenerAction() {
              @Override
              public void onValueChanged(Object val) {
                if (val != null) {
                  if (val instanceof Integer) {
                    motorDriveLeft.setMode(RUN_TO_POSITION);
                    motorDriveLeft.setTargetPosition((Integer) val);
                    motorDriveLeft.setPower(0.5);
                    while (motorDriveLeft.isBusy()) {

                    }
                    motorDriveLeft.setMode(RUN_USING_ENCODERS);
                  } else {
                    telemetry.addData("IntegerException", val + "is not an Integer");
                  }
                } else {
                  telemetry.addData("NullException", "The input is Null");
                }
              }
            }))
        .add(new ValueListener("motor.Drive Right",
            new ListenerAction() {
              @Override
              public void onValueChanged(Object val) {
                if (val != null) {
                  if (val instanceof Integer) {
                    motorDriveRight.setMode(RUN_TO_POSITION);
                    motorDriveRight.setTargetPosition((Integer) val);
                    motorDriveRight.setPower(0.5);
                    while (motorDriveRight.isBusy()) {

                    }
                    motorDriveRight.setMode(RUN_USING_ENCODERS);
                  } else {
                    telemetry.addData("IntegerException", val + "is not an Integer");
                  }
                } else {
                  telemetry.addData("NullException", "The input is Null");
                }
              }
            }))
        .add(new ValueListener("motor.Bucket Arm",
            new ListenerAction() {
              @Override
              public void onValueChanged(Object val) {
                if (val != null) {
                  if (val instanceof Integer) {
                    motorBucketArmRotation.setMode(RUN_TO_POSITION);
                    motorBucketArmRotation.setTargetPosition((Integer) val);
                    motorBucketArmRotation.setPower(0.3);
                    while (motorBucketArmRotation.isBusy()) {

                    }
                    motorBucketArmRotation.setMode(RUN_USING_ENCODERS);
                  } else {
                    telemetry.addData("IntegerException", val + "is not an Integer");
                  }
                } else {
                  telemetry.addData("NullException", "The input is Null");
                }
              }
            }))
        .add(new ValueListener("motor.Hang Arm",
            new ListenerAction() {
              @Override
              public void onValueChanged(Object val) {
                if (val != null) {
                  if (val instanceof Integer) {
                    motorHangArmRotation.setMode(RUN_TO_POSITION);
                    motorHangArmRotation.setTargetPosition((Integer) val);
                    motorHangArmRotation.setPower(0.3);
                    while (motorHangArmRotation.isBusy()) {

                    }
                    motorHangArmRotation.setMode(RUN_USING_ENCODERS);
                  } else {
                    telemetry.addData("IntegerException", val + "is not an Integer");
                  }
                } else {
                  telemetry.addData("NullException", "The input is Null");
                }
              }
            }))
        .add(new ValueListener("servo.Brush",
            new ListenerAction() {
              @Override
              public void onValueChanged(Object val) {
                if (val != null) {
                  if (val instanceof Double) {
                    brush.setPosition((Double) val);
                  } else {
                    telemetry.addData("DoubleException", val + "is not a Double");
                  }
                } else {
                  telemetry.addData("NullException", "The input is Null");
                }
              }
            }))
        .add(new ValueListener("servo.Arm Left",
            new ListenerAction() {
              @Override
              public void onValueChanged(Object val) {
                if (val != null) {
                  if (val instanceof Double) {
                    brushArmLeft.setPosition((Double) val);
                  } else {
                    telemetry.addData("DoubleException", val + "is not a Double");
                  }
                } else {
                  telemetry.addData("NullException", "The input is Null");
                }
              }
            }))
        .add(new ValueListener("servo.Arm Right",
            new ListenerAction() {
              @Override
              public void onValueChanged(Object val) {
                if (val != null) {
                  if (val instanceof Double) {
                    brushArmRight.setPosition((Double) val);
                  } else {
                    telemetry.addData("DoubleException", val + "is not a Double");
                  }
                } else {
                  telemetry.addData("NullException", "The input is Null");
                }
              }
            }))
        .add(new ValueListener("servo.Flap Left",
            new ListenerAction() {
              @Override
              public void onValueChanged(Object val) {
                if (val != null) {
                  if (val instanceof Double) {
                    flapLeft.setPosition((Double) val);
                  } else {
                    telemetry.addData("DoubleException", val + "is not a Double");
                  }
                } else {
                  telemetry.addData("NullException", "The input is Null");
                }
              }
            }))
        .add(new ValueListener("servo.Flap Right",
            new ListenerAction() {
              @Override
              public void onValueChanged(Object val) {
                if (val != null) {
                  if (val instanceof Double) {
                    flapRight.setPosition((Double) val);
                  } else {
                    telemetry.addData("DoubleException", val + "is not a Double");
                  }
                } else {
                  telemetry.addData("NullException", "The input is Null");
                }
              }
            }))
    ;


    // Start
    this.robotValues.start();
  }
}



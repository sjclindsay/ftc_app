 /* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.robotcontroller.external.samples;

 import android.app.Activity;
 import android.content.Context;
 import android.graphics.Color;
 import android.hardware.Sensor;
 import android.hardware.SensorEvent;
 import android.hardware.SensorEventListener;
 import android.hardware.SensorManager;
 import android.view.View;

 import com.qualcomm.ftcrobotcontroller.R;
 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.ColorSensor;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
 import com.qualcomm.robotcore.hardware.DigitalChannelController;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.hardware.TouchSensor;
 import com.qualcomm.robotcore.util.ElapsedTime;
 import com.qualcomm.robotcore.util.Range;

 import org.firstinspires.ftc.robotcore.external.Func;

 @TeleOp(name="WeCo: TeleOpAll", group="WeCo")
 //@Disabled
 public class WeCoTeleOpAll extends OpMode implements SensorEventListener {
     private enum TurnDir {
         LEFT,RIGHT
     }
     private enum ScaleStatus {
         OFF, UP, DOWN
     }

     //Servo Setup
     private Servo servoLeftRight;
     private Servo servoPushButton;
     private Servo servoUpDown;
     private static double SERVOLEFTRIGHT_STARTPOSITION = 0.5;
     private static double SERVOUPDOWN_STARTPOSITION = 0.2;
     private static double SERVOPUSHBUTTON_STARTPOSITION = 0.1;

     final static double servoMinRange = 0.0;
     final static double servoMaxRange = 1.0;
     double servoDelta = 0.25;
     private double servoLeftRightPosition = SERVOLEFTRIGHT_STARTPOSITION;
     private double servoUpDownPosition = SERVOUPDOWN_STARTPOSITION;
     private double servoPositionPushButon = SERVOPUSHBUTTON_STARTPOSITION;

     // bPrevState and bCurrState represent the previous and current state of the button.
     private static boolean bPrevState = false;
     // bLedOn represents the state of the LED.
     private boolean bLedOn = false;
     boolean BbuttonOn = false;
     ElapsedTime BbuttonTimmer = new ElapsedTime();
     boolean AbuttonOn = false;
     ElapsedTime AbuttonTimmer = new ElapsedTime();
     double buttonResetTime = 0.25;
     private ColorSensor sensorRGB;
     private DeviceInterfaceModule cdim;
     // we assume that the LED pin of the RGB sensor is connected to
     // digital port 5 (zero indexed).
     private static final int LED_CHANNEL = 0;
     // hsvValues is an array that will hold the hue, saturation, and value information.
     private float hsvValues[] = {0F, 0F, 0F};
     // values is a reference to the hsvValues array.
     final float values[] = hsvValues;

     // orientation values
     private float[] mGravity;       // latest sensor values
     private float[] mGeomagnetic;   // latest sensor values
     private float azimuth = 0.0f;      // value in radians

     private float pitch = 0.0f;        // value in radians
     private float roll = 0.0f;
     private String startDate;
     private SensorManager mSensorManager;
     private Sensor accelerometer;
     private Sensor magnetometer;

     //Setup Motor Spinner
     private DcMotor motorSpinner;
     private double motorSpinnerPower = 0.0;
     //Setup Motor values
     private DcMotor motorLeft1;
     private DcMotor motorLeft2;
     private DcMotor motorRight1;
     private DcMotor motorRight2;
     private TouchSensor touchSensor;

     private double motorLeft1power;
     private double motorLeft2power;
     private double motorRight1power;
     private double motorRight2power;
     private float motorPowerMin = -1;
     private float motorPowerMax = 1;
     private float motorScalar = 1;
     private ScaleStatus dpadPosition = ScaleStatus.OFF;
     int scaleNum = 0;
     private float wheelDiameter = 4;
     private double positionLeft;
     private double positionRight;
     private boolean prevLeftTriggerState = false;
     private boolean prevRightTriggerState = false;
     private boolean prevLeftBumperState = false;
     private boolean prevRightBumperState = false;


     @Override
     public void init() {
         // Init Servo Gimble
         servoLeftRight = hardwareMap.servo.get("servoLeftRightP1");
         servoUpDown = hardwareMap.servo.get("servoUpDownP2");
         servoPushButton = hardwareMap.servo.get("servoButtonP3");
         servoLeftRightPosition = SERVOLEFTRIGHT_STARTPOSITION;
         servoUpDownPosition = SERVOUPDOWN_STARTPOSITION;
         servoPositionPushButon = SERVOPUSHBUTTON_STARTPOSITION;

         // Init Color sensor
         cdim = hardwareMap.deviceInterfaceModule.get("dim");
         // set the digital channel to output mode.
         // remember, the Adafruit sensor is actually two devices.
         // It's an I2C sensor and it's also an LED that can be turned on or off.
         cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
         sensorRGB = hardwareMap.colorSensor.get("sensor_color");

         // turn the LED on in the beginning, just so user will know that the sensor is active.
         cdim.setDigitalChannelState(LED_CHANNEL, bLedOn);
         SetupTelemetry();
     }


     @Override
     public void start() {

         motorSpinner = hardwareMap.dcMotor.get("motorSpinP2");
         motorSpinner.setDirection(DcMotor.Direction.REVERSE);

         motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
         motorLeft2 = hardwareMap.dcMotor.get( "motorLeft2");
         motorRight1 = hardwareMap.dcMotor.get("motorRight1");
         motorRight2 = hardwareMap.dcMotor.get("motorRight2");
         motorRight1.setDirection(DcMotor.Direction.REVERSE);
         motorRight2.setDirection(DcMotor.Direction.REVERSE);
         touchSensor = hardwareMap.touchSensor.get("touchSensorP1");

         servoLeftRightPosition = SERVOLEFTRIGHT_STARTPOSITION;
         servoUpDownPosition = SERVOUPDOWN_STARTPOSITION;
         servoPositionPushButon = SERVOPUSHBUTTON_STARTPOSITION;

         mSensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
         accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
         magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

         servoLeftRight.setPosition(servoLeftRightPosition);
         servoPushButton.setPosition(servoPositionPushButon);
         servoUpDown.setPosition(servoUpDownPosition);

     }

     @Override
     public void loop() {
         //Set Spinner power
         motorSpinnerPower = setMotorSpinnerPower(gamepad2.left_trigger, gamepad2.left_bumper, 1);
         motorSpinnerPower += setMotorSpinnerPower(gamepad2.right_trigger, gamepad2.right_bumper, 2);

         //sets motor power
         motorScalar = getMotorScalar(gamepad1.dpad_up, gamepad1.dpad_down, motorScalar);
         motorLeft1power = controlmotor(gamepad1.right_stick_x, gamepad1.left_stick_y, motorScalar, TurnDir.LEFT);
         motorRight1power = controlmotor(gamepad1.right_stick_x, gamepad1.left_stick_y, motorScalar, TurnDir.RIGHT);
         motorLeft2power = motorLeft1power;
         motorRight2power = motorRight1power;

         //sets motor and servo power/position
         motorLeft1.setPower(motorLeft1power);
         motorLeft2.setPower(motorLeft2power);
         motorRight1.setPower(motorRight1power);
         motorRight2.setPower(motorRight2power);
         motorSpinner.setPower(motorSpinnerPower);

         // gets current position and uses formula to find rotations or distance in inches
         positionLeft = -motorLeft1.getCurrentPosition();
         positionLeft = (positionLeft / 2500);//(wheelDiameter*3.14159265358)
         positionRight = motorRight1.getCurrentPosition();
         positionRight = (positionRight / 2500); //(wheelDiameter*3.14159265358)

         // Sensor Control
        controlColorSensor(gamepad2.x);

         //Gimble Control
         servoPositionPushButon = controlButtonPusher(gamepad2.a, servoPositionPushButon);
         servoLeftRightPosition = controlGimble(-gamepad2.right_stick_x, servoLeftRightPosition, 200);
         servoUpDownPosition = controlGimble(-gamepad2.right_stick_y, servoUpDownPosition, 200);

         servoLeftRight.setPosition(servoLeftRightPosition);
         servoUpDown.setPosition(servoUpDownPosition);
         servoPushButton.setPosition(servoPositionPushButon);

         if (touchSensor.isPressed())
             telemetry.addData("Touch", "Is Pressed");
         else
             telemetry.addData("Touch", "Is Not Pressed");

         telemetry.update();
     }

     @Override
     public void stop() {
         servoLeftRight = hardwareMap.servo.get("servoLeftRightP1");
         servoUpDown = hardwareMap.servo.get("servoUpDownP2");
         servoPushButton = hardwareMap.servo.get("servoButtonP3");
         //servoLeftRight.setPosition(0.0);
         //servoPushButton.setPosition(0.0);
         //servoUpDown.setPosition(0.0);
         //cdim.setDigitalChannelState(LED_CHANNEL, false);
         //motorLeft1.setPower(0.0);
         //motorLeft2.setPower(0.0);
         //motorRight1.setPower(0.0);
         //motorRight2.setPower(0.0);
         //motorSpinner.setPower(0.0);

         //servoLeftRight.setPosition(SERVOLEFTRIGHT_STARTPOSITION);
         //servoPushButton.setPosition(SERVOPUSHBUTTON_STARTPOSITION);
         //servoUpDown.setPosition(SERVOUPDOWN_STARTPOSITION);
         mSensorManager.unregisterListener(this);
     }

     public void onAccuracyChanged(Sensor sensor, int accuracy) {
         // not sure if needed, placeholder just in case
     }

     public void onSensorChanged(SensorEvent event) {
         // we need both sensor values to calculate orientation
         // only one value will have changed when this method called, we assume we can still use the other value.
         if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
             mGravity = event.values;
         }
         if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
             mGeomagnetic = event.values;
         }
         if (mGravity != null && mGeomagnetic != null) {  //make sure we have both before calling getRotationMatrix
             float R[] = new float[9];
             float I[] = new float[9];
             boolean success = SensorManager.getRotationMatrix(R, I, mGravity, mGeomagnetic);
             if (success) {
                 float orientation[] = new float[3];
                 SensorManager.getOrientation(R, orientation);
                 azimuth = orientation[0]; // orientation contains: azimuth, pitch and roll
                 pitch = orientation[1];
                 roll = orientation[2];
             }
         }
     }

     private double setMotorSpinnerPower(double triggerValue, boolean bumperValue,  int scale) {
         triggerValue = bumperValue ? -1 * triggerValue/scale : triggerValue/scale;
         triggerValue = Range.clip(triggerValue, motorPowerMin, motorPowerMax);
         return triggerValue;
     }
     private float getMotorScalar(boolean scaleUp, boolean scaleDown, float Scalar) {

         if (!scaleUp && !scaleDown) {
             dpadPosition = ScaleStatus.OFF;
         }

         if (scaleUp && dpadPosition == ScaleStatus.OFF) {
             Scalar = Scalar / 2;
             dpadPosition = ScaleStatus.UP;
         }

         if (scaleDown && dpadPosition == ScaleStatus.OFF) {
             Scalar = Scalar * 2;
             dpadPosition = ScaleStatus.DOWN;
         }
         Scalar = Range.clip(Scalar, 1, 8) ;
         return Scalar;
     }

     private double controlGimble(double Stick, double Position, double Scale) {
         Position += Stick / Scale;
         Position = Range.clip(Position, servoMinRange, servoMaxRange);
         return Position;
     }

     private double controlmotor(double stickX, double stickY, float Scalar, TurnDir LeftRight) {
         double motorPower = 0.0;
         if(LeftRight == TurnDir.LEFT) {
             motorPower = (stickY - stickX);
         } else if (LeftRight == TurnDir.RIGHT) {
             motorPower = (stickY + stickX);
         }
         //goes slower if turning
         if (stickX != 0) {
             motorPower = 2 * (motorPower) / 3;
         }
         motorPower = motorPower / Scalar;
         motorPower = Range.clip(motorPower, motorPowerMin, motorPowerMax);
         return motorPower;
     }

     private double controlButtonPusher(boolean triggerPosition,double pushedPosition) {
         if(triggerPosition) {
             pushedPosition += 0.1;
         } else {
             pushedPosition = SERVOPUSHBUTTON_STARTPOSITION;
         }
         pushedPosition = Range.clip(pushedPosition, servoMinRange, servoMaxRange/2) ;
         return pushedPosition;
     }

     private void controlColorSensor(boolean bCurrState) {
         // check for button-press state transitions.
         if (bCurrState && bCurrState != bPrevState)  {
             // button is transitioning to a pressed state. Toggle the LED.
             bLedOn = !bLedOn;
             cdim.setDigitalChannelState(LED_CHANNEL, bLedOn);
         }
         // update previous state variable.
         bPrevState = bCurrState;
         // convert the RGB values to HSV values.
         Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);
     }

     private void SetupTelemetry() {

         telemetry.addLine()
                 .addData("MotorLT1Power", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f", motorLeft1power);
                     }
                 })
                 .addData("MotorLT2Power", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f", motorLeft2power);
                     }
                 });
         telemetry.addLine()
                 .addData("MotorRT1Power", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f", motorRight1power);
                     }
                 })
                 .addData("MotorRT2Power", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f", motorRight2power);
                     }
                 });
         telemetry.addLine()
                 .addData("Control Scalar:", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%f ", motorScalar);
                     }
                 });
         telemetry.addLine()
                 .addData("GamePad1 RT Stick_x", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f", gamepad1.right_stick_x);
                     }
                 })
                 .addData("LT Stick_y", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f", gamepad1.left_stick_y);
                     }
                 });
         telemetry.addLine()
                 .addData("Distance LT:", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f rotations", positionLeft);
                     }
                 })
                 .addData("RT:", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f rotations", positionRight);
                     }
                 });
         telemetry.addLine()
                 .addData("Spinner Motor Power", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f", motorSpinnerPower);
                     }
                 });
         telemetry.addLine()
                 .addData("GamePad1 Trig RT",  new Func<String>() {
                     @Override
                     public String value() {return String.format("%.2f", gamepad1.right_trigger) ; }
                 })
                 .addData("LT", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f", gamepad1.left_trigger);
                     }
                 });
         telemetry.addLine()
                 .addData("Servo LT/RT", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f", servoLeftRightPosition);
                     }
                 })
                 .addData("Up/Dn", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f", servoUpDownPosition);
                     }
                 })
                 .addData("Button", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f", servoPositionPushButon);
                     }
                 });
         telemetry.addLine()
                 .addData("GamePad2 Stick_x", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f", gamepad2.right_stick_x);
                     }
                 })
                 .addData("Stick_y", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f", gamepad2.right_stick_y);
                     }
                 });
         telemetry.addLine()
                 .addData(" GamePad2 Trig RT", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f", gamepad2.right_trigger);
                     }
                 })
                 .addData(" LT", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f", gamepad2.left_trigger);
                     }
                 });
         telemetry.addLine()
                 .addData("LED", bLedOn ? "On" : "Off")
                 .addData("Hue", hsvValues[0]);
         telemetry.addLine()
                 .addData("Red ", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%d", sensorRGB.red());
                     }
                 })
                 .addData("Green ", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%d", sensorRGB.green());
                     }
                 });
         telemetry.addLine()
                 .addData("Blue ", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%d", sensorRGB.blue());
                     }
                 })
                 .addData("Clear ", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%d", sensorRGB.alpha());
                     }
                 });
         telemetry.addLine()
                 .addData("X", String.valueOf(gamepad2.x));

         telemetry.addLine()
                .addData("Right Bumper", new Func<String>() {
                    @Override
                    public String value() {return String.valueOf( gamepad1.right_bumper) ; }
                })
                 .addData("Trigger", new Func<String>() {
                     @Override
                     public String value() {return String.format("%.2f", gamepad1.right_trigger) ; }
                 });
         telemetry.addLine()
                 .addData("Left Bumper", new Func<String>() {
                     @Override
                     public String value() {return String.valueOf(gamepad1.left_bumper) ; }
                 })
                 .addData("Trigger", new Func<String>() {
                    @Override
                    public String value() {return String.format("%.2f", gamepad1.left_trigger) ; }
                });
/*         telemetry.addLine()
                 .addData("Gyro Z", new Func<String>() {
                     @Override
                     public String value() {return String.valueOf( gamepad1.right_bumper) ; }
                 })
                 .addData("Left Trigger", new Func<String>() {
                     @Override
                     public String value() {return String.format("%.2f", gamepad1.left_trigger) ; }
                 })
                 .addData("Left Bumper", new Func<String>() {
                     @Override
                     public String value() {return String.format("%.2f", gamepad1.left_bumper) ; }
                 });
 */

         /*
         telemetry.addLine()
                 .addData("Phone Gyro Azmith ", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f", Math.round(Math.toDegrees(azimuth)));
                     }
                 })
                 .addData("Pitch ", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f", Math.round(Math.toDegrees(pitch)));
                     }
                 })
                 .addData("Roll ", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f", Math.round(Math.toDegrees(roll)));
                     }
                 })
                 */
         ;
     }
 }

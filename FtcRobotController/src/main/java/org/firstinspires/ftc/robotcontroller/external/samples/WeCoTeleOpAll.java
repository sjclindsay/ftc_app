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
 import com.qualcomm.robotcore.util.ElapsedTime;
 import com.qualcomm.robotcore.util.Range;

 import org.firstinspires.ftc.robotcore.external.Func;

 @TeleOp(name="WeCo: TeleOpAll", group="WeCo")
 //@Disabled
 public class WeCoTeleOpAll extends OpMode implements SensorEventListener {
     public enum TurnDir {
         LEFT,RIGHT
     }
     public enum ScaleStatus {
         OFF, UP, DOWN
     }

     //Servo Setup
     Servo servoLeftRight;
     Servo servoPushButton;
     Servo servoUpDown;
     private static double SERVOLEFTRIGHT_STARTPOSITION = 0.5;
     private static double SERVOUPDOWN_STARTPOSITION = 0.2;
     private static double SERVOPUSHBUTTON_STARTPOSITION = 0.0;

     final static double servoMinRange = 0.0;
     final static double servoMaxRange = 1.0;
     double servoDelta = 0.25;
     double servoLeftRightPosition;
     double servoUpDownPosition;
     double servoPositionPushButon;

     // bPrevState and bCurrState represent the previous and current state of the button.
     private static boolean bPrevState = false;
     private static boolean bCurrState = false;
     // bLedOn represents the state of the LED.
     private boolean bLedOn = false;
     boolean BbuttonOn = false;
     ElapsedTime BbuttonTimmer = new ElapsedTime();
     boolean AbuttonOn = false;
     ElapsedTime AbuttonTimmer = new ElapsedTime();
     double buttonResetTime = 0.25;
     ColorSensor sensorRGB;
     DeviceInterfaceModule cdim;
     // we assume that the LED pin of the RGB sensor is connected to
     // digital port 5 (zero indexed).
     static final int LED_CHANNEL = 0;
     // hsvValues is an array that will hold the hue, saturation, and value information.
     float hsvValues[] = {0F, 0F, 0F};
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
     Sensor accelerometer;
     Sensor magnetometer;

     //Setup Motor values
     DcMotor motorLeft1;
     DcMotor motorLeft2;
     DcMotor motorRight1;
     DcMotor motorRight2;

     double motorLeft1power;
     double motorLeft2power;
     double motorRight1power;
     double motorRight2power;
     float motorPowerMin = -1;
     float motorPowerMax = 1;
     float motorScalar = 1;
     int scaleNum = 0;
     float wheelDiameter = 4;
     double positionLeft;
     double positionRight;

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
         // hsvValues is an array that will hold the hue, saturation, and value information.
         float hsvValues[] = {0F, 0F, 0F};
         // values is a reference to the hsvValues array.
         final float values[] = hsvValues;
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

         motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
         motorLeft2 = hardwareMap.dcMotor.get( "motorLeft2");
         motorRight1 = hardwareMap.dcMotor.get("motorRight1");
         motorRight2 = hardwareMap.dcMotor.get("motorRight2");
         motorLeft1.setDirection(DcMotor.Direction.REVERSE);
         motorLeft2.setDirection(DcMotor.Direction.REVERSE);

         servoLeftRightPosition = SERVOLEFTRIGHT_STARTPOSITION;
         servoUpDownPosition = SERVOUPDOWN_STARTPOSITION;
         servoPositionPushButon = SERVOPUSHBUTTON_STARTPOSITION;

         mSensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
         accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
         magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);


     }

     @Override
     public void loop() {
         //sets motor power
         motorScalar = getMotorScalar(gamepad1.dpad_up, gamepad1.dpad_down, motorScalar);
         motorLeft1power = controlmotor(gamepad1.left_stick_x, -gamepad1.left_stick_y, motorScalar, TurnDir.LEFT);
         motorRight1power = controlmotor(gamepad1.left_stick_x, -gamepad1.left_stick_y, motorScalar, TurnDir.RIGHT);
         motorLeft2power = motorLeft1power;
         motorRight2power = motorRight1power;

         //sets motor and servo power/position
         motorLeft1.setPower(motorLeft1power);
         motorLeft2.setPower(motorLeft2power);
         motorRight1.setPower(motorRight1power);
         motorRight2.setPower(motorRight2power);

         // gets current position and uses formula to find rotations or distance in inches
         positionLeft = -motorLeft1.getCurrentPosition();
         positionLeft = (positionLeft / 2500);//(wheelDiameter*3.14159265358)
         positionRight = motorRight1.getCurrentPosition();
         positionRight = (positionRight / 2500); //(wheelDiameter*3.14159265358)

         // Sensor Control
        controlColorSensor();

         //Gimble Control
         servoPositionPushButon = controlButtonPusher(gamepad2.right_trigger, servoPositionPushButon);
         servoLeftRightPosition = controlGimble(gamepad2.right_stick_x, servoLeftRightPosition, 200);
         servoUpDownPosition = controlGimble(-gamepad2.right_stick_y, servoUpDownPosition, 200);

         servoLeftRight.setPosition(servoLeftRightPosition);
         servoUpDown.setPosition(servoUpDownPosition);
         servoPushButton.setPosition(servoPositionPushButon);

         telemetry.update();
     }

     @Override
     public void stop() {
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

     private float getMotorScalar(boolean scaleUp, boolean scaleDown, float Scalar) {
         ScaleStatus dpadPosition = ScaleStatus.OFF;

         if (scaleUp == false && scaleDown == false) {
             dpadPosition = ScaleStatus.OFF;
         }

         if (gamepad1.dpad_up == true && dpadPosition.equals("Off")) {
             Scalar = Scalar / 2;
             dpadPosition = ScaleStatus.UP;
         }

         if (gamepad1.dpad_down == true && dpadPosition.equals("Off")) {
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

     private double controlButtonPusher(double triggerPosition,double pushedPosition) {
         if(triggerPosition > 0.0) {
             pushedPosition += triggerPosition/10;
         } else {
             pushedPosition = SERVOPUSHBUTTON_STARTPOSITION;
         }
         pushedPosition = Range.clip(pushedPosition, servoMinRange, servoMaxRange) ;
         return pushedPosition;
     }

     public void controlColorSensor() {
         bCurrState = gamepad2.x;
         // check for button-press state transitions.
         if ((bCurrState == true) && (bCurrState != bPrevState))  {
             // button is transitioning to a pressed state. Toggle the LED.
             bLedOn = !bLedOn;
             cdim.setDigitalChannelState(LED_CHANNEL, bLedOn);
         }
         // update previous state variable.
         bPrevState = bCurrState;
         // convert the RGB values to HSV values.
         Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);
     }

     public void SetupTelemetry() {
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
                 .addData("Control Scalar:", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%f ", motorScalar);
                     }
                 });
         ;

         telemetry.addLine()
                 .addData(" Right Trigger", new Func<String>() {
                     @Override
                     public String value() {
                         return String.format("%.2f", gamepad2.right_trigger);
                     }
                 });
         telemetry.addLine()
                 .addData("Stick_x", new Func<String>() {
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
                 })
         ;
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
                 .addData("X", String.valueOf(gamepad2.x))
         ;
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
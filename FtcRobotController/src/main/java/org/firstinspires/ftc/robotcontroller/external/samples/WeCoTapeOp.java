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

 import android.content.Context;
 import android.graphics.Color;
 import android.hardware.Sensor;
 import android.hardware.SensorEvent;
 import android.hardware.SensorEventListener;
 import android.hardware.SensorManager;

 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.CRServo;
 import com.qualcomm.robotcore.hardware.ColorSensor;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
 import com.qualcomm.robotcore.hardware.DigitalChannelController;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.hardware.TouchSensor;
 import com.qualcomm.robotcore.util.ElapsedTime;
 import com.qualcomm.robotcore.util.Range;

 import org.firstinspires.ftc.robotcore.external.Func;

 @TeleOp(name="WeCo: TapeOp", group="WeCo")
 //@Disabled
 public class WeCoTapeOp extends OpMode {
     private enum TurnDir {
         LEFT,RIGHT
     }
     private enum ScaleStatus {
         OFF, UP, DOWN
     }

     //Setup Tape Measure
     private CRServo servoTapeRight;
     private CRServo servoTapeLeft;
     private static double SERVOTAPERIGHT_STARTPOSITION = 0.0;
     private static double SERVOTAPERCLEFT_STARTPOSITION = 0.0;
     private double servoTapeRightSpeed;
     private double servoTapeLeftSpeed;

     final static double servoMinRange = 0.0;
     final static double servoMaxRange = 1.0;
     private double servoTapeLeftPosition;
     private double servoTapeRightPosition;

     private float motorPowerMin = -1;
     private float motorPowerMax = 1;
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
         //Init Servo Tape
         servoTapeLeft = hardwareMap.crservo.get("servoTapeLeft");
         servoTapeRight = hardwareMap.crservo.get("servoTapeRight");

         SetupTelemetry();
     }

     @Override
     public void start() {

         servoTapeLeft.setPower(SERVOTAPERCLEFT_STARTPOSITION);
         servoTapeRight.setPower(SERVOTAPERIGHT_STARTPOSITION);
     }

     @Override
     public void loop() {
         //Tape Extend
         servoTapeLeftPosition = setServoTapeExtension(gamepad1.left_trigger, gamepad1.left_bumper);
         servoTapeRightPosition = setServoTapeExtension(gamepad1.right_trigger, gamepad1.right_bumper);


         servoTapeLeft.setPower(servoTapeLeftPosition);
         servoTapeRight.setPower(servoTapeRightPosition);

         telemetry.update();
     }

     @Override
     public void stop() {
         servoTapeRight.setPower(0.0);
         servoTapeLeft.setPower(0.0);
     }

     public double setServoTapeExtension(double trigger, boolean bumper) {
         double position;
         if(trigger > 0.0) {
             position = 1.0;
         }else if(bumper){
             position = -1.0;
         } else {
             position =0.0;
         }
         return( position);

     }
     public void onAccuracyChanged(Sensor sensor, int accuracy) {
         // not sure if needed, placeholder just in case
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

     private void SetupTelemetry() {
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

     }
 }
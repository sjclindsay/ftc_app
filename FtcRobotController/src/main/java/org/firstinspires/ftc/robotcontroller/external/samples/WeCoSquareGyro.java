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

 import com.qualcomm.hardware.adafruit.BNO055IMU;
 import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.LightSensor;
 import com.qualcomm.robotcore.hardware.TouchSensor;
 import com.qualcomm.robotcore.util.Range;

 import org.firstinspires.ftc.robotcore.external.Func;
 import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
 import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
 import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
 import org.firstinspires.ftc.robotcore.external.navigation.Position;
 import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

 import java.util.Locale;

 @Autonomous(name="WeCo: AutoSquareGyro", group="WeCo")
// @Disabled
 public class WeCoSquareGyro extends OpMode  {
   // Initialize HW Data Objects
   TouchSensor touchSensor1;  // Hardware Device Object
   LightSensor lightSensor1;  // Hardware Device Object
   // The IMU sensor object
   BNO055IMU imu;

   // State used for updating telemetry
   Orientation angles;
   Acceleration gravity;

   DcMotor motorLeft1;
   DcMotor motorLeft2;
   DcMotor motorRight1 ;
   DcMotor motorRight2 ;

   //Drive Control Values
   static final float normalTurnSpeed = (float) 0.75;
   static final float normalSpeed = (float) 0.75;
   static final float normalLine = 1;
   static final double normalturn = 0.5;
   float resetValueLeft, resetValueRight = 0;
   float motorPowerMin = -1;
   float motorPowerMax = 1;
   double positionLeft, positionRight;
   float motorLeft1Power = 0;
   float motorLeft2Power = 0;
   float motorRight1Power = 0;
   float motorRight2Power = 0;

   public WeCoSquareGyro() {
   }

   @Override
   public void init() {

     // get a reference to our Hardware objects
     touchSensor1 = hardwareMap.touchSensor.get("touchSensor1");
     lightSensor1 = hardwareMap.lightSensor.get("lightSensor1");
     motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
     motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
     motorRight1 = hardwareMap.dcMotor.get("motorRight1");
     motorRight2 = hardwareMap.dcMotor.get("motorRight2");

     //Setup Hardware
     motorLeft1.setDirection(DcMotor.Direction.REVERSE);
     motorLeft2.setDirection(DcMotor.Direction.REVERSE);

     // Set up the parameters with which we will use our IMU. Note that integration
     // algorithm here just reports accelerations to the logcat log; it doesn't actually
     // provide positional information.
     BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
     parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
     parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
     parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
     parameters.loggingEnabled      = true;
     parameters.loggingTag          = "IMU";
     // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

     // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
     // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
     // and named "imu".
     imu = hardwareMap.get(BNO055IMU.class, "imu");
     imu.initialize(parameters);

     whattodo = 1;
     count = 0;
     composeTelemetry();
   }
  //Event Control Value
   double count;
   double etime;
   int whattodo = 1; //0 do nothing; 1 moveforward; 2 turn

   @Override
   public void start() {
     lightSensor1.enableLed(true);
     imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

   }

   @Override
   public void loop() {

     telemetry.update();
     if(touchSensor1.isPressed())
           whattodo = 6;
     switch (whattodo) {
       case 1:
         resetValueLeft = -motorLeft1.getCurrentPosition();
         resetValueRight = motorRight1.getCurrentPosition();
         moveForward();
         whattodo = 2;
         break;
       case 2:
           // gets current position and uses formula to find rotations or distance in inches
           positionLeft = -motorLeft1.getCurrentPosition() - resetValueLeft;
           positionRight = motorRight1.getCurrentPosition() - resetValueRight;
           positionLeft = (positionLeft / 2500);//(wheelDiameter*3.14159265358)
           positionRight = (positionRight / 2500); //(wheelDiameter*3.14159265358)

         if((Math.abs(positionLeft) > normalLine) && (positionRight > normalLine))
           whattodo =3;
         break;
       case 3:
         stopMove();
         resetValueLeft = -motorLeft1.getCurrentPosition();
         resetValueRight = motorRight1.getCurrentPosition();
         startLeftTurn();
         whattodo = 4;
         break;
       case 4:
         // gets current position and uses formula to find rotations or distance in inches
         positionLeft = -motorLeft1.getCurrentPosition()- resetValueLeft;
         positionRight = motorRight1.getCurrentPosition() - resetValueRight;

         positionLeft = (positionLeft / 2500);//(wheelDiameter*3.14159265358)
         positionRight = (positionRight / 2500); //(wheelDiameter*3.14159265358)
         if((Math.abs(positionLeft) > normalturn) && (positionRight > normalturn))
           whattodo = 5;
         break;
       case 5:
         stopMove();
         resetValueLeft = -motorLeft1.getCurrentPosition();
         resetValueRight = motorRight1.getCurrentPosition();
         count ++;
         if (count == 4) {
           whattodo = 6;
         }
         else {
           whattodo = 1;
         }
         break;
       case 6:
         stopMove();
         break;
       default:
         whattodo = 0;
         break;

     }
     //clips motor and servo power/position

       motorLeft1Power = Range.clip(motorLeft1Power, motorPowerMin, motorPowerMax);
       motorLeft2Power = Range.clip(motorLeft2Power, motorPowerMin, motorPowerMax);

       motorRight1Power = Range.clip(motorRight1Power, motorPowerMin, motorPowerMax);
       motorRight2Power = Range.clip(motorRight2Power, motorPowerMin, motorPowerMax);

     // servoHookPosition = Range.clip(servoHookPosition, servoMinRange, servoMaxRange);
     //servoCDPosition = Range.clip(servoCDPosition, servoMinRange, servoMaxRange);


     //sets motor and servo power/position
     motorLeft1.setPower(motorLeft1Power);
     motorLeft2.setPower(motorLeft2Power);
     motorRight1.setPower(motorRight1Power);
     motorRight2.setPower(motorRight2Power);
     // motorLifter.setPower(motorLifterPower);
     //servoHook.setPosition(servoHookPosition);
       //servoCD.setPosition(servoCDPosition);
       // servoTail.setPosition(servoTailPosition);


       // gets current position and uses formula to find rotations or distance in inches
       positionLeft = -motorLeft1.getCurrentPosition();
       positionRight = motorRight1.getCurrentPosition();

       positionLeft = (positionLeft / 2500);  //(wheelDiameter*3.14159265358)
       positionRight = (positionRight / 2500); //(wheelDiameter*3.141592653)



       motorLeft1.setPower(motorLeft1Power);
   }

   void composeTelemetry() {

     // At the beginning of each telemetry update, grab a bunch of data
     // from the IMU that we will then display in separate lines.
     telemetry.addAction(new Runnable() { @Override public void run()
     {
       // Acquiring the angles is relatively expensive; we don't want
       // to do that in each of the three items that need that info, as that's
       // three times the necessary expense.
       angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
       gravity  = imu.getGravity();
     }
     });

     telemetry.addLine()
         .addData("status", new Func<String>() {
           @Override public String value() {
             return imu.getSystemStatus().toShortString();
           }
         })
         .addData("calib", new Func<String>() {
           @Override public String value() {
             return imu.getCalibrationStatus().toString();
           }
         });

     telemetry.addLine()
         .addData("heading", new Func<String>() {
           @Override public String value() {
             return formatAngle(angles.angleUnit, angles.firstAngle);
           }
         })
         .addData("roll", new Func<String>() {
           @Override public String value() {
             return formatAngle(angles.angleUnit, angles.secondAngle);
           }
         })
         .addData("pitch", new Func<String>() {
           @Override public String value() {
             return formatAngle(angles.angleUnit, angles.thirdAngle);
           }
         });

     telemetry.addLine()
         .addData("Light1 Raw", new Func<String>() {
           @Override public String value() {
             return formatDouble(lightSensor1.getRawLightDetected());
           }
         })
         .addData("Normal", new Func<String>() {
           @Override public String value() {
             return formatDouble(lightSensor1.getLightDetected());
           }
         });
     telemetry.addLine()
         .addData("Motor Power Left1", new Func<String>() {
           @Override public String value() {
             return formatDegrees(motorLeft1Power);
           }
         })
         .addData("Left2", new Func<String>() {
           @Override
           public String value() {
             return formatDouble(motorLeft2Power);
           }
         })
         .addData("Right1", new Func<String>() {
           @Override
           public String value() {
             return formatDouble(motorRight1Power);
           }
         })
         .addData("Right2", new Func<String>() {
           @Override
           public String value() {
             return formatDouble(motorRight2Power);
           }
         });
     telemetry.addLine()
         .addData("Position Left", new Func<String>() {
           @Override public String value() {
             return formatDouble(positionLeft);
           }
         })
         .addData("Right", new Func<String>() {
           @Override public String value() {
             return formatDouble(positionRight);
           }
         });
     telemetry.addLine()
         .addData("Control Count", new Func<String>() {
           @Override public String value() {
             return String.format("%.2f", count);
           }
         })
         .addData("WhatToDo", new Func<String>() {
           @Override public String value() {
             return String.format("%.2f",whattodo));
           }
         });
     ;
   }

   //----------------------------------------------------------------------------------------------
   // Formatting
   //----------------------------------------------------------------------------------------------

   String formatDouble(double inputValue) {
     return String.format("%.2f", inputValue)
   }

   String formatAngle(AngleUnit angleUnit, double angle) {
     return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
   }

   String formatDegrees(double degrees){
     return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
   }
 }

   public void startLeftTurn(){
     motorLeft1Power = -normalTurnSpeed;
     motorLeft2Power = -normalTurnSpeed;
     motorRight1Power = normalTurnSpeed;
     motorRight2Power = normalTurnSpeed;
   }
   public void moveForward(){
     motorLeft1Power = normalSpeed;
     motorLeft2Power = normalSpeed;
     motorRight1Power = normalSpeed;
     motorRight2Power = normalSpeed;
   }
   public void stopMove(){
     motorLeft1Power = 0;
     motorLeft2Power = 0;
     motorRight1Power = 0;
     motorRight2Power = 0;
   }

   public float motorPosition(DcMotor motor, float resetValue) {
     return(motor.getCurrentPosition() - resetValue) ;
   }

   @Override
   public void stop() {
   }

 }
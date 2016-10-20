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

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */

@TeleOp(name="WeCo: ServoOp", group="WeCo")
//@Disabled
public class ServoOp extends OpMode {
    Servo servoLeftRight;
    Servo servoPushButton;


    final static double servoMinRange = 0.0 ;
    final static double servoMaxRange = 1.0 ;
    double servoDelta = 0.25 ;
    double servoLeftRightPosition;
    double servoPositionPushButon;
    boolean BbuttonOn= false;
    ElapsedTime BbuttonTimmer = new ElapsedTime();
    boolean RTriggerOn= false;
    ElapsedTime AbuttonTimmer = new ElapsedTime();
    double buttonResetTime = 0.25 ;


    @Override
  public void init() {
        servoLeftRight = hardwareMap.servo.get("servo_1") ;
        servoPushButton = hardwareMap.servo.get("servo_2");

  }

    public void start() {
        servoLeftRightPosition = 0.5 ;
        servoPositionPushButon = 0.0;
    }

  @Override
  public void loop() {

      servoPositionPushButon += gamepad2.right_trigger/100;
      if(gamepad2.right_trigger == 0.0) {
          servoPositionPushButon = 0.0;
      }

      servoLeftRightPosition += -gamepad2.right_stick_y;

      //DbgLog.msg("=====servoPosition====="+String.format("%f", servoPositionClipped)) ;
      servoPositionPushButon = Range.clip(servoPositionPushButon, servoMinRange, servoMaxRange) ;
      servoLeftRightPosition = Range.clip(servoLeftRightPosition, servoMinRange, servoMaxRange) ;
      servoPushButton.setPosition(servoPositionPushButon);
      servoLeftRight.setPosition(servoLeftRightPosition);

      telemetry.addData("Left Trigger", "Left Trigger is at " + String.format("%.2f", gamepad2.left_trigger));
      telemetry.addData("Right Trigger", "Right Trigger is at " + String.format("%.2f", gamepad2.right_trigger));
      telemetry.addData("Servo Position", "Servo is at " + String.format("%f", servoPositionPushButon)) ;

  }
  @Override
  public void stop() {

  }
}


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
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;

import java.security.SecureRandom;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad

 */


@TeleOp(name="WeCo: ServoOp", group="WeCo")
@Disabled
public class ServoOp extends OpMode {
    Servo servoLeftRight;
    Servo servoPushButton;
    Servo servoUpDown;
    private static double SERVOLEFTRIGHT_STARTPOSITION = 0.5;
    private static double SERVOUPDOWN_STARTPOSITION = 0.2;
    private static double SERVOPUSHBUTTON_STARTPOSITION = 1.0;

    final static double servoMinRange = 0.0 ;
    final static double servoMaxRange = 1.0 ;
    double servoDelta = 0.25 ;
    double servoLeftRightPosition;
    double servoUpDownPosition;
    double servoPositionPushButon;

    // bPrevState and bCurrState represent the previous and current state of the button.
    private static boolean bPrevState = false;
    private static boolean bCurrState = false;
    // bLedOn represents the state of the LED.
    private boolean bLedOn = false;
    boolean BbuttonOn= false;
    ElapsedTime BbuttonTimmer = new ElapsedTime();
    boolean AbuttonOn= false;
    ElapsedTime AbuttonTimmer = new ElapsedTime();
    double buttonResetTime = 0.25 ;

    ColorSensor sensorRGB;
    DeviceInterfaceModule cdim;
    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).
    static final int LED_CHANNEL = 0;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    @Override
  public void init() {
        servoLeftRight = hardwareMap.servo.get("servoLeftRightP1") ;
        servoUpDown = hardwareMap.servo.get("servoUpDownP2");
        servoPushButton = hardwareMap.servo.get("servoButtonP3");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        // get a reference to our DeviceInterfaceModule object.
        cdim = hardwareMap.deviceInterfaceModule.get("dim");

        // set the digital channel to output mode.
        // remember, the Adafruit sensor is actually two devices.
        // It's an I2C sensor and it's also an LED that can be turned on or off.
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        // get a reference to our ColorSensor object.
        sensorRGB = hardwareMap.colorSensor.get("sensor_color");

        // turn the LED on in the beginning, just so user will know that the sensor is active.
        cdim.setDigitalChannelState(LED_CHANNEL, bLedOn);
        servoLeftRightPosition = SERVOLEFTRIGHT_STARTPOSITION;
        servoUpDownPosition = SERVOUPDOWN_STARTPOSITION;
        servoPositionPushButon = SERVOPUSHBUTTON_STARTPOSITION;
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
                .addData("Servo Up/Dn", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format("%.2f", servoUpDownPosition);
                    }
                });
        telemetry.addLine()
                .addData("LED", bLedOn ? "On" : "Off")
                .addData("Hue", hsvValues[0]);
        telemetry.addLine()
                .addData("Red ", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format("%d",sensorRGB.red());
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
                        return String.format("%d",sensorRGB.blue());
                    }
                })
                .addData("Clear ", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format("%d", sensorRGB.alpha());
                    }
                });
        telemetry.addLine()
            .addData("X",String.valueOf(gamepad2.x))
        ;
    }

    public void start() {
        servoLeftRightPosition = SERVOLEFTRIGHT_STARTPOSITION;
        servoUpDownPosition = SERVOUPDOWN_STARTPOSITION;
        servoPositionPushButon = SERVOPUSHBUTTON_STARTPOSITION;

    }
    
  @Override
  public void loop() {


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

      if(gamepad2.right_trigger > 0.0) {
          servoPositionPushButon -= gamepad2.right_trigger/10;
      } else {
          servoPositionPushButon = SERVOPUSHBUTTON_STARTPOSITION;
      }

      servoLeftRightPosition += (gamepad2.right_stick_x/200);
      servoUpDownPosition += (-gamepad2.right_stick_y/200);
      servoPositionPushButon = Range.clip(servoPositionPushButon, servoMinRange, servoMaxRange) ;

      servoLeftRight.setPosition(servoLeftRightPosition);
      servoUpDown.setPosition(servoUpDownPosition);
      servoPushButton.setPosition(servoPositionPushButon);

      telemetry.update();


  }
  @Override
  public void stop() {

  }
}


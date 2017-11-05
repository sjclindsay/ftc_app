package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by matth on 9/16/2017.
 */

public class HardwareJewel {

    Servo servoJewel = null;
    HardwareColorSensor jewelSensor;


    public static float servoJewelMaxPosition = (float) 1.0;
    public static float servoJewelInitalPosition = (float) 1.0;

    HardwareMap hwMap = null;

    /* Constructor */
    public HardwareJewel() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        servoJewel = hwMap.servo.get("servoJewel") ;
        servoJewel.setPosition(servoJewelInitalPosition);


        jewelSensor = new HardwareColorSensor();
        jewelSensor.init(hwMap);
    }

    public void start() {

    }

    //put functions here

    public void setServoJewelPosition(double position) {

        position = position * 0.3 + 0.5;

        position = Range.clip(position, 0.5, servoJewelMaxPosition);

        servoJewel.setPosition(position);
    }
    public HardwareColorSensor.Color WhatColor (){
        return jewelSensor.WhatColor();
    }

    public void led_off() {
        jewelSensor.sensorLED.setState(false);
    }

    public void led_on() {
        jewelSensor.sensorLED.setState(true);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine()
                .addData("ServoJewel ", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatDouble(servoJewel.getPosition());
                    }
                })
                .addData("Color ", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatColor(jewelSensor.WhatColor());
                    }
                })
                .addData("Red ", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatDouble(jewelSensor.colorSensor.red());
                    }
                })
                .addData("Blue ", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatDouble(jewelSensor.colorSensor.blue());
                    }
                })
                .addData("Green ", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatDouble(jewelSensor.colorSensor.green());
                    }
                });
    }
}
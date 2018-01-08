package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.util.RobotLog;


public class HardwareColorSensor {

    private int RED_Threshold = 30000;
    private int BLUE_Threshold = 30000;
    public PWMOutput sensorLED = null;
    ColorSensor colorSensor;   // Hardware Device Object
    HardwareMap hwMap = null;

    public void init (HardwareMap ahwMap, HardwareColorSensor.Color targColor){
        String smackerSensorName = "";
        String smackerLEDName = "";
        hwMap = ahwMap ;

        if(targColor == Color.Red) {
            smackerSensorName = "SmackerColorSensor";
            smackerLEDName = "LED";
        } else if(targColor == Color.Blue) {
            smackerSensorName = "SmackerColorSensorBlue";
            smackerLEDName = "LEDBlue";
        } else {
            RobotLog.e("ERROR: No target Color passed");
        }

        sensorLED = hwMap.pwmOutput.get(smackerLEDName);
        sensorLED.setPulseWidthPeriod(20000);
        sensorLED.setPulseWidthOutputTime(10);


        colorSensor = hwMap.colorSensor.get (smackerSensorName) ;
    }

    public enum Color {
        Blue,
        Red,
        None
    }

    public Color WhatColor (){

        RobotLog.i("color blue " + colorSensor.blue()) ;
        RobotLog.i("color red " + colorSensor.red()) ;

        if (colorSensor.red() > colorSensor.blue()) {
            return Color.Red ;
        }
        else if (colorSensor.blue() >colorSensor.red()) {
            return Color.Blue;
        }
        else {
            return Color.None;
        }
    }
}
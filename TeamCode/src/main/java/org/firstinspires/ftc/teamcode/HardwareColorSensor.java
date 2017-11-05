package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class HardwareColorSensor {

    private int RED_Threshold = 100;
    private int BLUE_Threshold = 100;
    public DigitalChannel sensorLED = null;
    ColorSensor colorSensor;   // Hardware Device Object
    HardwareMap hwMap = null;

    public void init (HardwareMap ahwMap){
        hwMap = ahwMap ;

        sensorLED = hwMap.digitalChannel.get("SmackerLED");
        sensorLED.setMode(DigitalChannel.Mode.OUTPUT);
        sensorLED.setState(false);

        colorSensor = hwMap.colorSensor.get ("SmackerColorSensor") ;
    }

    public enum Color {
        Blue,
        Red,
        None
    }

    public Color WhatColor (){
        if (colorSensor.red() > RED_Threshold) {
            return Color.Red ;
        }
        else if (colorSensor.blue() >BLUE_Threshold) {
            return Color.Blue;
        }
        else {
            return Color.None;
        }
    }
}
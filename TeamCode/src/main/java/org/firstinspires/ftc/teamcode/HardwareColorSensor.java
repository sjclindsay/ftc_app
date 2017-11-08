package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PWMOutput;


public class HardwareColorSensor {

    private int RED_Threshold = 30000;
    private int BLUE_Threshold = 30000;
    public PWMOutput sensorLED = null;
    ColorSensor colorSensor;   // Hardware Device Object
    HardwareMap hwMap = null;

    public void init (HardwareMap ahwMap){
        hwMap = ahwMap ;

        sensorLED = hwMap.pwmOutput.get("LED");
        sensorLED.setPulseWidthPeriod(20000);
        sensorLED.setPulseWidthOutputTime(0);

        colorSensor = hwMap.colorSensor.get ("SmackerColorSensor") ;
    }

    public enum Color {
        Blue,
        Red,
        None
    }

    public Color WhatColor (){
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
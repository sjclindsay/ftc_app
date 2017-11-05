package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class HardwareColorSensor {

    private DeviceInterfaceModule cdim;
    ColorSensor colorSensor;   // Hardware Device Object
    HardwareMap hwMap = null;

    public void init (HardwareMap ahwMap){
        hwMap = ahwMap ;

        cdim = hwMap.deviceInterfaceModule.get("dim");
        cdim.setLED(0,false);
        colorSensor = hwMap.colorSensor.get ("SmackerColorSensor") ;
    }

    public enum Color {
        Blue,
        Red,
        None
    }

    public Color WhatColor (){
        if (colorSensor.red() > 100) {
            return Color.Red ;
        }
        else if (colorSensor.blue() >100) {
            return Color.Blue;
        }
        else {
            return Color.None;
        }
    }
}
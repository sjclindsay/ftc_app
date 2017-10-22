package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class HardwareColorSensor {

    ColorSensor colorSensor;   // Hardware Device Object
    HardwareMap hwMap = null;

    public void init (HardwareMap ahwMap){
        hwMap = ahwMap ;
        colorSensor = hwMap.colorSensor.get ("SmackerColorSensor") ;
    }

    enum Color {
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
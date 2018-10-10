package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardwareColorSensor;

import java.util.Locale;

/**
 * This is NOT an opmode.
 *
 * THis Class is used to define a PID controller of any kind
 *
 */

public class FormatHelper
{
    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------
    public static String formatDouble(double inputValue) {
        return String.format("%.2f", inputValue);
    }

    public static String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public static String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public static String formatColor(Color currentColor) {
        if(currentColor == Color.Red) {
            return("RED");
        } else if(currentColor == Color.Blue){
            return("BLUE");
        } else {
            return ("None");
        }
    }

}


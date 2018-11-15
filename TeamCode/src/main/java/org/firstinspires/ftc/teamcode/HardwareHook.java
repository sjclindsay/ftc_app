package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by matth on 9/16/2017.
 */

public class HardwareHook {
    Servo hookServo = null ;
    public DigitalChannel sensorRetracted ;
    public DigitalChannel sensorExtended ;

    public boolean hookExtended = false ;
    public boolean hookRetracted = false ;
    public double hook = 0.0;
    public boolean hookClosed = false;
    public boolean hookOpened = false;

    HardwareMap hwMap = null;

    /* Constructor */
    public HardwareHook() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        sensorRetracted = hwMap.digitalChannel.get("retractedLimit") ;
        sensorExtended = hwMap.digitalChannel.get("extendedLimit") ;
    }

    public void start() {

    }

    //put functions here
    public void setHookSpeed (double hookSpeed) {
        closeLimit();
        openLimit();
        if (hookExtended && hookSpeed < 0.5) {
            hook = 0.5 ;
        }
        if (hookClosed && hookSpeed > 0.5) {
            hookSpeed = 0.5;
        }
            hookServo.setPosition(hookSpeed);
    }

    public void closeLimit () {
            hookClosed = true ;
    }
    public void openLimit () {
            hookOpened = true ;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine()
                .addData("ServoHook ", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatDouble(hookServo.getPosition());
                    }
                })
                .addData("Red ", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatDouble((hookServo.getPosition()));
                    }
                })
                ;
    }
}
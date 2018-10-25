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
    public DigitalChannel hookRetracted ;
    public DigitalChannel hookExtended ;

    public boolean hookOpened = false ;
    public boolean hookClosed = false ;


    HardwareMap hwMap = null;

    /* Constructor */
    public HardwareHook() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        hookServo = hwMap.servo.get("servoHook") ;
        hookRetracted = hwMap.digitalChannel.get("closeLimit") ;
        hookExtended = hwMap.digitalChannel.get("openLimit") ;

        hookServo.setPosition(0.5);
    }

    public void start() {

    }

    //put functions here
    public void setHookSpeed (double hookSpeed) {
        closeLimit();
        openLimit();
        if (hookOpened && hookSpeed < 0.5) {
            hookSpeed = 0.5 ;
        }
        if (hookClosed && hookSpeed > 0.5) {
            hookSpeed = 0.5;
        }
            hookServo.setPosition(hookSpeed);
    }

    public void closeLimit () {
        if (hookExtended.getState()) {
            hookClosed = true ;
        }
    }
    public void openLimit () {
        if (hookRetracted.getState()) {
            hookOpened = true ;
        }
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine()
                ;
    }
}
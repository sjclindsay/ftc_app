package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by matth on 9/16/2017.
 */

public class HardwareGrabber {

    Servo servoGrabber1 = null;
    Servo servoGrabber2 = null ;
    public static float servoGrabberInitialPosition = (float) 0.8 ;

    HardwareMap hwMap = null;

    /* Constructor */
    public HardwareGrabber() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        servoGrabber1 = hwMap.servo.get("servoGrabber1") ;
        servoGrabber2 = hwMap.servo.get("servoGrabber2") ;

        servoGrabber1.setPosition(0.5);
        servoGrabber2.setPosition(0.5);

    }

    public void start() {

    }

    //put functions here

    public void setServoGrabberPosition ( double position) {

        position = position*0.3 + 0.5 ;

        position = Range.clip(position, 0.5, servoGrabberInitialPosition) ;

        servoGrabber1.setPosition(position);
        servoGrabber2.setPosition(1 - position);

    }

}

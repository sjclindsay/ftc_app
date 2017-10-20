package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by matth on 9/16/2017.
 */

public class HardwareGrabber {

    Servo servoGrabber = null;
    public static float servoGrabberInitialPosition = (float) 0.85 ;

    HardwareMap hwMap = null;

    /* Constructor */
    public HardwareGrabber() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        servoGrabber = hwMap.servo.get("servoGrabber") ;

        servoGrabber.setPosition(servoGrabberInitialPosition);
    }

    public void start() {

    }

    //put functions here

    public void setServoGrabberPosition ( double position) {

        position = Range.clip(position, 0.5, servoGrabberInitialPosition) ;

        servoGrabber.setPosition(position);

    }

}

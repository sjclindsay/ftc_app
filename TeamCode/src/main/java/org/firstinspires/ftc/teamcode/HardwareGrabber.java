package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by matth on 9/16/2017.
 */

public class HardwareGrabber {

    Servo servoGrabber = null;

    HardwareMap hwMap = null;

    /* Constructor */
    public HardwareGrabber() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        servoGrabber = hwMap.servo.get("servoGrabber") ;

        servoGrabber.setPosition(0);
    }

    public void start() {

    }

    //put functions here

    public void setServoGrabberPosition ( double position) {

        servoGrabber.setPosition(position);

    }

}

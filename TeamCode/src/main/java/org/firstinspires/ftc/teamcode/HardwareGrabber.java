package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by matth on 9/16/2017.
 */

public class HardwareGrabber {

    Servo servoGrabber1 = null;
    Servo servoGrabber2 = null;
    public static float servoGrabberMaxPosition = (float) 0.8;
    public static float servoGrabberMinPosition = (float) 0.55;
    public static float servoGrabberStartPosition = servoGrabberMinPosition;

    HardwareMap hwMap = null;

    /* Constructor */
    public HardwareGrabber() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        servoGrabber1 = hwMap.servo.get("servoGrabber1");
        servoGrabber2 = hwMap.servo.get("servoGrabber2");

        servoGrabber1.setPosition(servoGrabberStartPosition);
        servoGrabber2.setPosition(1-servoGrabberStartPosition);

    }

    public void start() {

    }

    //put functions here

    public void setServoGrabberPosition(double position) {

        position = position * 0.3 + 0.5;

        position = Range.clip(position, 0.5, servoGrabberMaxPosition);

        servoGrabber1.setPosition(position);
        servoGrabber2.setPosition(1 - position);

    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine()
                .addData("Grabber1 ", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatDouble(servoGrabber1.getPosition());
                    }
                })
                .addData("Grabber2 ", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatDouble(servoGrabber2.getPosition());
                    }
                });
    }
}
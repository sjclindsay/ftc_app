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

public class HWTeamMarkerServo {

    Servo teamMarkerServo = null;
    DigitalChannel sensorMarkerDropped = null;
    public static double servoMarkerDrop = 0.4;
    public static double servoMarkerUp = 0.95;
    public static double servoMarkStartPosition = servoMarkerUp;

    HardwareMap hwMap = null;

    /* Constructor */
    public HWTeamMarkerServo() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        teamMarkerServo = hwMap.servo.get("servoMarkerTeam");
        teamMarkerServo.setPosition(servoMarkStartPosition);

        sensorMarkerDropped = hwMap.digitalChannel.get("sensorMarkerDropped") ;
    }

    public void start() {

    }

    //put functions here

    public void setTeamMarkerPosition(double position) {

        position = Range.clip(position, servoMarkerDrop, servoMarkerUp);
        teamMarkerServo.setPosition(position);
    }

    public boolean isTeamMarkerDropped() {
        return (!sensorMarkerDropped.getState());
    }

    public void dropTeamMarker() {
        teamMarkerServo.setPosition(servoMarkerDrop);
    }

    public void liftMarkerHolder() {
        teamMarkerServo.setPosition(servoMarkerUp);
    }

    public double getTeamMarkerServoPosition() {
        return(teamMarkerServo.getPosition());
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine()
                .addData("Team Marker ", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatDouble(teamMarkerServo.getPosition());
                    }
                });
    }
}
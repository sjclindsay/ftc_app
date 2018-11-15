package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by matth on 9/16/2017.
 */

enum LifterHWcontroller {
    Lifter,
    LifterGrabber,
}

public class HardwareLifter {

    public DcMotor motorLifter = null;
    public  DigitalChannel lifterRangeUpper ;
    public DigitalChannel lifterRangeLower ;

    private LifterHWcontroller connectedHW = LifterHWcontroller.Lifter;

    HardwareMap hwMap = null;

    /* Constructor */
    public HardwareLifter() {

    }

    public HardwareLifter(LifterHWcontroller connectedParts) {

    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

       // motorLifter = hwMap.dcMotor.get("motorLifter") ;
        lifterRangeLower = hwMap.digitalChannel.get("topLimit") ;
        lifterRangeUpper = hwMap.digitalChannel.get("bottomLimit") ;

    }

    public void start() {

    }

    //put functions here

    public  void setLifter (float lifterSpeed) {
        motorLifter.setPower(lifterSpeed);
    }


    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine()
                .addData("Lifter Power ", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatDouble(motorLifter.getPower());
                    }
                });
        telemetry.addLine()
                .addData("Limit Top", new Func<String>() {
                    @Override
                    public String value() {
                        return String.valueOf(lifterRangeUpper.getState());
                    }
                })
                .addData("Limit Bottom", new Func<String>() {
                    @Override
                    public String value() {
                        return String.valueOf(lifterRangeLower.getState());
                    }
                });
    }

}

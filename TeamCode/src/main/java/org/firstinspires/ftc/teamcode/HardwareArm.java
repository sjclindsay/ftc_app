package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by matth on 9/16/2017.
 */

public class HardwareArm {

    public DcMotor motorArm = null;

    HardwareMap hwMap = null;

    int targetPosition = 0 ;
    int targetPositionManual = 0 ;

    /* Constructor */
    public HardwareArm() {

    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        motorArm = hwMap.dcMotor.get("motorArm") ;
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorArm.setTargetPosition(0);

    }

    public void start() {

    }

    //put functions here
    public void stowArm() {
        targetPosition = 0 ;
    }
    public void raiseArm() {
        targetPosition = 140 ;
    }
    public void lowerArm() {
        targetPosition = 210 ;
    }

    public void manualArmControl(float stickPos) {
        targetPositionManual = targetPosition + 140 * (int)stickPos ;
        motorArm.setTargetPosition(targetPositionManual);
    }


    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine()
                .addData("Arm Power ", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatDouble(motorArm.getPower());
                    }
                });
        telemetry.addLine()
                .addData("Encoder at ", new Func<String>() {
                    @Override
                    public String value() {
                        return String.valueOf(motorArm.getCurrentPosition());
                    }
                });
    }

}

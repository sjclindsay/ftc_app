package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

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
    float armPower = 0 ;

    private PIDController armPID = null ;
    private double kp = 0.0055 ;
    private double ki = 0.00001 ;
    private double kd = 0 ;


    private int lastPosition = 1 ;
    private float correction = 0 ;

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
        targetPosition = -350 ;
    }
    public void lowerArm() {
        targetPosition = -1050 ;
    }

    public void manualArmControl(float stickPos, float power) {
        targetPositionManual = targetPosition + (int) (-200 * stickPos) ;

        motorArm.setPower(0.3);
        motorArm.setTargetPosition(targetPositionManual);

        /*
        //for use with out own PID controller
        armPower = getPower(targetPositionManual, power);
        if (Math.abs(targetPositionManual - motorArm.getCurrentPosition()) > 5) {
            motorArm.setPower(armPower);
        }
        */

    }

    float getPower(int position, float power) {

        if (position != lastPosition) {
            RobotLog.i("initialized PID") ;
            armPID = new PIDController(position, kp, ki, kd) ;
        }

        correction = (float) armPID.Update((double) motorArm.getCurrentPosition()) ;
        power = power + correction ;

        lastPosition = position ;

        return power ;
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
        telemetry.addData("Target Power ", new Func<String>() {
            @Override
            public String value() {
                return FormatHelper.formatDouble(targetPositionManual);
            }
        });
    }

}

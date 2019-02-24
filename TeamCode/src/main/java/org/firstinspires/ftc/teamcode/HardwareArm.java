package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by matth on 9/16/2017.
 */

public class HardwareArm {

    public DcMotor motorArm = null;
    public DigitalChannel armCalibrate = null ;
    public Servo servoExtender = null ;

    HardwareMap hwMap = null;

    int targetPosition = 0 ;
    int targetPositionManual = 0 ;
    int lastTargetPosition = 0 ;
    static final int halfPosition = -700 ;
    boolean topTHRToggle = false ;
    float armPower = 0 ;
    boolean calibrateToggle = false ;

    ElapsedTime armTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS) ;


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

        armCalibrate = hwMap.digitalChannel.get("armLimit") ;
        servoExtender = hwMap.servo.get("servoExtender") ;

        motorArm.setTargetPosition(0);
        servoExtender.setPosition(0.5);

    }

    public void start() {

    }

    public void calibrateArmUp() {
        if (armCalibrate.getState()) {
            targetPosition = 10000 ;
            motorArm.setPower(0.1);
            motorArm.setTargetPosition(10000);
            calibrateToggle = false ;
        } else if (!calibrateToggle && !armCalibrate.getState()) {
            motorArm.setPower(0);
            motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            calibrateToggle = true ;
        } else {

        }
    }
    public void calibrateArmDown() {
        if (armCalibrate.getState()) {
            targetPosition = -10000 ;
            motorArm.setPower(-0.1);
            motorArm.setTargetPosition(targetPosition);
            calibrateToggle = false ;
        } else if (!calibrateToggle && !armCalibrate.getState()) {
            motorArm.setPower(0);
            motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            calibrateToggle = true ;
        } else {

        }
    }

    //put functions here
    public void stowArm() {
        targetPosition = 0 ;
    }
    public void raiseArm() {
        targetPosition = -400 ;
    }
    public void lowerArm() {
        targetPosition = -700 ;
    }

    public void manualArmControl(float stickPos, float power) {
        targetPositionManual = targetPosition + (int) (-200 * stickPos) ;

        motorArm.setPower(0.3);
        motorArm.setTargetPosition(targetPositionManual);
/*
        if ((lastTargetPosition > halfPosition) || (lastTargetPosition < halfPosition)) {
            armTime.reset();
            topTHRToggle = true ;
        }
        if (topTHRToggle) {
            motorArm.setTargetPosition(halfPosition);
            if (armTime.time() > 500) {
                topTHRToggle = false ;
            }
        } else {
            motorArm.setTargetPosition(targetPositionManual);
        }

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

    void setServoExtender(float extendo, float retracto) {
        float servoPower = (float)0.52 ;
        servoPower = servoPower + extendo - retracto ;
        servoPower = Range.clip(servoPower, 0, 1) ;
        servoExtender.setPosition(servoPower);
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

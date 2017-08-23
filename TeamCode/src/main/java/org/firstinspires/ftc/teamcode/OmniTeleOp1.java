package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by conno on 8/17/2017.
 */

@TeleOp(name="Omni: TeleOp1", group="Omni")


public class OmniTeleOp1 extends OpMode {

    DcMotor motorLeft1;
    DcMotor motorLeft2;
    DcMotor motorRight1 ;
    DcMotor motorRight2 ;

    float motorLeft1power;
    float motorLeft2power;
    float motorRight1power;
    float motorRight2power;

    float motorPowerMin = -1 ;
    float motorPowerMax = 1 ;

    @Override
    public void init() {
        motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight1 = hardwareMap.dcMotor.get("motorRight1");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");

        motorRight1.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        gamepad1.left_stick_y = -gamepad1.left_stick_y ;

        motorLeft1power = gamepad1.left_stick_y + gamepad1.left_stick_x ;
        motorLeft2power = gamepad1.left_stick_y - gamepad1.left_stick_x ;
        motorRight1power = gamepad1.left_stick_y - gamepad1.left_stick_x ;
        motorRight2power = gamepad1.left_stick_y + gamepad1.left_stick_x ;

        motorLeft1power = Range.clip(motorLeft1power, motorPowerMin, motorPowerMax);
        motorLeft2power = Range.clip(motorLeft2power, motorPowerMin, motorPowerMax);
        motorRight1power = Range.clip(motorRight1power, motorPowerMin, motorPowerMax);
        motorRight2power = Range.clip(motorRight2power, motorPowerMin, motorPowerMax);

        motorLeft1.setPower(motorLeft1power);
        motorLeft2.setPower(motorLeft2power);
        motorRight1.setPower(motorRight1power);
        motorRight2.setPower(motorRight2power);

    }

    @Override
    public void stop() {

    }

}

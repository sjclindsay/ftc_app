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

    float motorLeft1power = 0;
    float motorLeft2power = 0;
    float motorRight1power = 0;
    float motorRight2power = 0;
    float leftStickY = 0 ;
    HardwareOmniBot OmniBot ;

    @Override
    public void init() {
        OmniBot = new HardwareOmniBot() ;
        OmniBot.init(hardwareMap);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        leftStickY = -gamepad1.left_stick_y ;

        motorLeft1power = leftStickY  + gamepad1.left_stick_x + gamepad1.right_stick_x;
        motorLeft2power = leftStickY  - gamepad1.left_stick_x + gamepad1.right_stick_x;
        motorRight1power = leftStickY - gamepad1.left_stick_x - gamepad1.right_stick_x;
        motorRight2power = leftStickY + gamepad1.left_stick_x - gamepad1.right_stick_x;

        OmniBot.setBotMovement(motorLeft1power, motorLeft2power, motorRight1power, motorRight2power);

        OmniBot.waitForTick(40);
    }

    @Override
    public void stop() {

    }

}

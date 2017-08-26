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
import org.firstinspires.ftc.teamcode.FormatHelper;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

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
    boolean controller1 = true;
    boolean controller2 = false ;
    HardwareOmniBot OmniBot ;

    @Override
    public void init() {
        OmniBot = new HardwareOmniBot() ;
        OmniBot.init(hardwareMap);
        composeTelemetry();
    }

    @Override
    public void start() {
        OmniBot.start();
    }

    @Override
    public void loop() {
        leftStickY = -gamepad1.left_stick_y ;

        if (gamepad1.a || controller1) {
            controller1 = true ;
            controller2 = false ;

            motorLeft1power = leftStickY  + gamepad1.left_stick_x + gamepad1.right_stick_x;
            motorLeft2power = leftStickY  - gamepad1.left_stick_x + gamepad1.right_stick_x;
            motorRight1power = leftStickY - gamepad1.left_stick_x - gamepad1.right_stick_x;
            motorRight2power = leftStickY + gamepad1.left_stick_x - gamepad1.right_stick_x;

            OmniBot.setBotMovement(motorLeft1power, motorLeft2power, motorRight1power, motorRight2power);

        }
        if (gamepad2.a || controller2) {
            controller2 = true ;
            controller1 = false ;
            OmniBot.complexOmniBotMath(-gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x );
        }

        OmniBot.waitForTick(40);
        telemetry.update()
    }

    @Override
    public void stop() {

    }
    void composeTelemetry() {

        OmniBot.getTelemetry(telemetry);
    }




}

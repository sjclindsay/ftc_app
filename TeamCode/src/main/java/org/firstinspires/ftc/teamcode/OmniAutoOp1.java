package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by conno on 8/17/2017.
 */

@TeleOp(name="Omni: AutoOp1", group="Omni")


public class OmniAutoOp1 extends OpMode {
    public enum MotorState{
        Drive,
        Turn,
        InitializeTurn
    }
    MotorState currentState = MotorState.Turn;
    float leftStickY = 0 ;
    boolean controller1 = true;
    boolean controller2 = false ;
    robotHWconnected autoConnectedHW = robotHWconnected.MotorLifterColorCrypto;
    HardwareOmniBot OmniBot ;
    ElapsedTime StabilizationTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void init() {
        OmniBot = new HardwareOmniBot(autoConnectedHW) ;
        OmniBot.init(hardwareMap);

        int count = 0;

        composeTelemetry();
    }

    @Override
    public void start() {

        OmniBot.start();
    }

    @Override
    public void loop() {
        leftStickY = -gamepad1.left_stick_y ;

        if ((gamepad1.a && !controller1) || (gamepad2.a && !controller2)) {
            OmniBot.resetFirstPIDDrive();
        }

        if (gamepad1.a || controller1) {
            controller1 = true ;
            controller2 = false ;

            OmniBot.lowerServoJewel();
            OmniBot.crypto.lowerCryptoServo();

            if (OmniBot.jewelSystem.jewelSensor.WhatColor() == HardwareColorSensor.Color.Red) {
                RobotLog.i("red") ;
            } else if (OmniBot.jewelSystem.jewelSensor.WhatColor() == HardwareColorSensor.Color.Blue) {
                RobotLog.i("blue") ;
            } else {
                RobotLog.i("color not found") ;
            }

        }
        if (gamepad2.a || controller2) {
            controller2 = true;
            controller1 = false;

            OmniBot.raiseServoJewel();
            OmniBot.crypto.raiseCryptoServo();

        }
        OmniBot.waitForTick(40);
        telemetry.update();

    }

    @Override
    public void stop() {

    }
    void composeTelemetry() {

        OmniBot.addTelemetry(telemetry);
        //OmniBot.getTelemetry(telemetry);
    }




}

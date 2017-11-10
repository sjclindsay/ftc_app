package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Func;

/**
 * Created by conno on 8/17/2017.
 */

@Autonomous(name="Omni: AutoJewelRed", group="Omni")
//@Disable
public class OmniAutoJewelRed extends OpMode {
    public enum MotorState{
        WAIT_START,
        CHECK_COLOR,
        TURN_COUNTERCLOCKWISE,
        TURN_CLOCKWISE,
        STOP_MOVING,
        HitWait,
        DELAY,
        ERROR_STATE
    }
    MotorState currentState = MotorState.ERROR_STATE;
    float targetHeading = 0 ;
    float magnitude = 0 ;
    float direction = 90 ;
    int delay_time = 0;
    double currentHeading = 0 ;
    robotHWconnected autoConnectedHW = robotHWconnected.MotorGyroLifterCryptoJewel;
    HardwareOmniBot OmniBot ;
    ElapsedTime StabilizationTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double last_time = 0;
    ElapsedTime WaitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double waitTimer  = 0 ;
    MotorState nextState = MotorState.WAIT_START;
    MotorState stateAfterNext = MotorState.ERROR_STATE ;

    @Override
    public void init() {
        OmniBot = new HardwareOmniBot(autoConnectedHW) ;
        OmniBot.init(hardwareMap);

        int count = 0;
        currentState = MotorState.WAIT_START;
        nextState = MotorState.WAIT_START;
        composeTelemetry();
    }

    @Override
    public void start() {

        OmniBot.start();
    }

    @Override
    public void loop() {
        waitTimer = WaitTimer.time() ;
        if(currentState != nextState) {
            RobotLog.i("Change State to " + nextState);
        }


        currentHeading = OmniBot.getcurrentHeading() ;

        currentState = nextState;

        switch(nextState) {
            case DELAY:
                if(waitTimer - last_time >= delay_time) {
                    nextState = stateAfterNext;
                }
            case WAIT_START:
                OmniBot.jewelSystem.setJewelDown();
                nextState = MotorState.DELAY;
                delay_time = 1000;
                stateAfterNext = MotorState.CHECK_COLOR;
                last_time = WaitTimer.milliseconds();
                break;
            case CHECK_COLOR:
                RobotLog.i("In CHECK_COLOR");
                RobotLog.i("Found " + OmniBot.jewelSystem.WhatColor());
                OmniBot.jewelSystem.led_low();
                if(OmniBot.jewelSystem.WhatColor() == HardwareColorSensor.Color.Red) {
                    //OmniBot.Red_LEDon();
                    nextState = MotorState.TURN_COUNTERCLOCKWISE;
                } else if (OmniBot.jewelSystem.WhatColor()== HardwareColorSensor.Color.Blue) {
                    //OmniBot.Blue_LEDon();
                    nextState = MotorState.TURN_CLOCKWISE;
                }

                //if (StabilizationTimer.time() > 5000) {
                //    nextState = stateAfterNext ;
                //}
                break;
            case TURN_COUNTERCLOCKWISE:
                targetHeading = (float) (currentHeading + 10.0);
                OmniBot.driveOmniBot(0,0,targetHeading,PIDAxis.gyro);
                nextState = MotorState.HitWait ;
                WaitTimer.reset();
                break;
            case TURN_CLOCKWISE:
                targetHeading = (float) (currentHeading - 10.0);
                OmniBot.driveOmniBot(0,0,targetHeading,PIDAxis.gyro);
                nextState = MotorState.HitWait ;
                WaitTimer.reset();
                break;
            case HitWait:
                if (WaitTimer.time() > 500){
                    nextState = MotorState.STOP_MOVING   ;
                } else if (currentHeading == targetHeading) {
                    nextState = MotorState.STOP_MOVING;
                }
                break;
            case STOP_MOVING:
                OmniBot.setBotMovement(0,0,0,0);
                break;
            case ERROR_STATE:
                RobotLog.i("Error_State");
            default:
                OmniBot.setBotMovement(0,0,0,0);
                RobotLog.i("error no case");
                break;
        }

        telemetry.update();
        OmniBot.waitForTick(40);
    }

    @Override
    public void stop() {
        //OmniBot.Red_LEDoff();
        //OmniBot.Blue_LEDoff();
        OmniBot.jewelSystem.led_off();
        OmniBot.setBotMovement(0,0,0,0);

    }
    void composeTelemetry() {
        telemetry.addLine()
                .addData("Current State ", new Func<String>() {
                    @Override
                    public String value() {
                        return currentState.name();
                    }
                })
                .addData("Next State", new Func<String>() {
                    @Override
                    public String value() {
                        return nextState.name();
                    }
                }) ;
        telemetry.addLine()
                .addData("Wait Timer ", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatDouble(WaitTimer.milliseconds());
                    }
                })
                .addData("Stabilize Timer", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatDouble(StabilizationTimer.milliseconds());
                    }
                }) ;
        OmniBot.addTelemetry(telemetry);

    }




}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by conno on 8/17/2017.
 */

@TeleOp(name="Omni: AutoJewelRed", group="Omni")


public class OmniAutoJewelRed extends OpMode {
    public enum MotorState{
        WAIT_START,
        CHECK_COLOR,
        TURN_COUNTERCLOCKWISE,
        TURN_CLOCKWISE,
        STOP_MOVING,
        HitWait,
        ERROR_STATE
    }
    MotorState currentState = MotorState.ERROR_STATE;
    float targetHeading = 0 ;
    float magnitude = 0 ;
    float direction = 90 ;
    double currentHeading = 0 ;
    robotHWconnected autoConnectedHW = robotHWconnected.MotorGyroLifterCryptoJewel;
    HardwareOmniBot OmniBot ;
    ElapsedTime StabilizationTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
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
        if(currentState != nextState) {
            RobotLog.i("Change State to " + nextState);
        }

        waitTimer = WaitTimer.time() ;
        currentHeading = OmniBot.getcurrentHeading() ;

        currentState = nextState;

        switch(nextState) {
            case WAIT_START:
                OmniBot.jewelSystem.setJewelDown();
                nextState = MotorState.CHECK_COLOR;
                break;
            case CHECK_COLOR:
                OmniBot.jewelSystem.led_on();
                if(OmniBot.jewelSystem.WhatColor() == HardwareColorSensor.Color.Red) {
                    nextState = MotorState.TURN_COUNTERCLOCKWISE;
                } else if (OmniBot.jewelSystem.WhatColor()== HardwareColorSensor.Color.Blue) {
                    nextState = MotorState.TURN_CLOCKWISE;
                }
                  else {
                    nextState = MotorState.ERROR_STATE ;
                }

                if (StabilizationTimer.time() > 500) {
                    nextState = stateAfterNext ;
                }
                break;
            case TURN_COUNTERCLOCKWISE:
                OmniBot.driveOmniBot(0,0,10,PIDAxis.gyro);
                nextState = MotorState.HitWait ;
                WaitTimer.reset();
                break;
            case TURN_CLOCKWISE:
                OmniBot.driveOmniBot(0,0,-10,PIDAxis.gyro);
                nextState = MotorState.HitWait ;
                WaitTimer.reset();
            break;
            case HitWait:
                if (WaitTimer.time() > 500){
                    nextState = MotorState.STOP_MOVING   ;
                }
                break;

            case STOP_MOVING:
                OmniBot.driveOmniBot(0, 0, (float)currentHeading, PIDAxis.gyro);
                break;
            case ERROR_STATE:
                RobotLog.i("Error_State");
            default:
                OmniBot.driveOmniBot(0, 0, (float)currentHeading, PIDAxis.gyro);
                RobotLog.i("error no case");
                break;
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

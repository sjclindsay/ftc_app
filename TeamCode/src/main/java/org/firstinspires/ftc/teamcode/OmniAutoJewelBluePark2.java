package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Func;

/**
 * Created by conno on 8/17/2017.
 */

@Autonomous(name="Omni: AutoJewelBluePark2", group="Omni")
@Disabled
public class OmniAutoJewelBluePark2 extends OpMode {
    public enum MotorState{
        WAIT_START,  //0
        CHECK_COLOR,  //1
        TURN_COUNTERCLOCKWISE,  //2
        TURN_CLOCKWISE,  //3
        STOP_MOVING,  //4
        HitWait, //5
        DELAY,  //6
        INITIALIZE, //7
        INITIALIZEDRIVEOFFPLATFORM,  //8
        DRIVEOFFPLATFORM,  //9
        DRIVETOSAFEZONE,  //10
        STOPROBOT,  //11
        WAIT,  //12
        ERROR_STATE  //13
    }
    MotorState currentState = MotorState.ERROR_STATE;
    float targetHeading = 0 ;
    float magnitude = 0 ;
    float direction = 90 ;
    double delay_time = 0;
    double currentHeading = 0 ;
    robotHWconnected autoConnectedHW = robotHWconnected.MotorGyroLifterCryptoJewel;
    HardwareOmniBot OmniBot ;
    ElapsedTime StabilizationTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double last_time = 0;
    double current_delay = 0;
    ElapsedTime WaitTimer = new  ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime FreeRunningTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double waitTimer  = 0 ;
    MotorState nextState = MotorState.WAIT_START;
    MotorState stateAfterNext = MotorState.ERROR_STATE ;

    @Override
    public void init() {
        OmniBot = new HardwareOmniBot(autoConnectedHW) ;
        OmniBot.init(hardwareMap, Color.Blue);

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


        currentHeading = OmniBot.getcurrentHeading() ;

        currentState = nextState;

        switch(nextState) {
            case DELAY:
                current_delay = getRuntime() - last_time;
                //RobotLog.i("Delay Time " + current_delay);
                if(current_delay >= delay_time) {
                    nextState = stateAfterNext;
                }
                break;
            case WAIT_START:
                OmniBot.jewelSystem.lowerServoJewel();
                nextState = MotorState.DELAY;
                delay_time = 0.5;
                stateAfterNext = MotorState.CHECK_COLOR;
                last_time = getRuntime();
                break;
            case CHECK_COLOR:
                RobotLog.i("In CHECK_COLOR");
                RobotLog.i("Found " + OmniBot.jewelSystem.WhatColor());
                OmniBot.jewelSystem.led_low();
                if(OmniBot.jewelSystem.WhatColor() == Color.Blue) {
                    //OmniBot.Red_LEDon();
                    nextState = MotorState.TURN_COUNTERCLOCKWISE;
                } else if (OmniBot.jewelSystem.WhatColor()== Color.Red) {
                    //OmniBot.Blue_LEDon();
                    nextState = MotorState.TURN_CLOCKWISE;
                }
                break;
            case TURN_COUNTERCLOCKWISE:
                targetHeading = (float) (currentHeading + 100.0);
                OmniBot.driveOmniBot(0,0,targetHeading,PIDAxis.gyro);
                WaitTimer.reset();
                nextState = MotorState.HitWait;
                break;
            case TURN_CLOCKWISE:
                targetHeading = (float) (currentHeading - 100.0);
                RobotLog.i("Start Turn " + currentHeading);
                OmniBot.driveOmniBot(0,0,targetHeading,PIDAxis.gyro);
                WaitTimer.reset();
                nextState = MotorState.HitWait ;
                break;
            case HitWait:
                if (WaitTimer.time() > 1000){
                    nextState = MotorState.STOP_MOVING   ;
                } else if (Math.abs(currentHeading - targetHeading) <= 3) {
                    RobotLog.i("Reach Target Heading" + targetHeading);
                    nextState = MotorState.STOP_MOVING;
                }
                break;
            case STOP_MOVING:
                OmniBot.jewelSystem.raiseServoJewel();
                OmniBot.crypto.lowerCryptoServo();
                OmniBot.setBotMovement(0,0,0,0);
                WaitTimer.reset();
                nextState = MotorState.WAIT ;
                stateAfterNext = MotorState.INITIALIZEDRIVEOFFPLATFORM;
                break;
            case INITIALIZEDRIVEOFFPLATFORM:
                //red side (i think)
                OmniBot.setBotMovement((double) -0.3, (double) -0.3, (double) -0.3, (double) -0.3);
                if ( Math.abs(OmniBot.gyroScope.currentHeadingY) >= 2.5) {
                    nextState = MotorState.DRIVEOFFPLATFORM ;
                }
                break;
            case DRIVEOFFPLATFORM:
                if (Math.abs(OmniBot.gyroScope.currentHeadingY) <= 2.5 ) {
                    OmniBot.setBotMovement(-0.1, -0.1, -0.1, -0.1);
                    WaitTimer.reset();
                    nextState = MotorState.WAIT ;
                    stateAfterNext = MotorState.DRIVETOSAFEZONE;
                }
                break;
            case DRIVETOSAFEZONE:
                OmniBot.setBotMovement(-0.1, -0.1, -0.1, -0.1);
                if (Math.abs(OmniBot.gyroScope.currentAccelerationY) >= 100 || Math.abs(OmniBot.gyroScope.currentAccelerationX) >= 100 || StabilizationTimer.time() >= 5000) {
                        OmniBot.setBotMovement(0, 0, 0, 0);
                    nextState = MotorState.STOPROBOT ;
                }
                break;
            case STOPROBOT:
                OmniBot.setBotMovement(0, 0, 0, 0);
                RobotLog.i("robot stopped") ;
                break;
            case WAIT:
                if (WaitTimer.time() >= 500) {
                    nextState = stateAfterNext ;
                    StabilizationTimer.reset();
                }
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
                .addData("State" , new Func<String>() {
                    @Override
                    public String value() {
                        return String.valueOf(currentState);
                    }
                })
                .addData("current ", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatDouble(current_delay);
                    }
                })
                .addData("last ", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatDouble(last_time);
                    }
                });
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

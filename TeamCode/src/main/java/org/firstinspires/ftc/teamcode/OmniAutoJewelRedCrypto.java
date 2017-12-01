package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Func;

/**
 * Created by conno on 8/17/2017.
 */

@Autonomous(name="Omni: AutoJewelRedCrypto", group="Omni")
//@Disable
public class OmniAutoJewelRedCrypto extends OpMode {
    public enum MotorState{
        WAIT_START,
        CHECK_COLOR,
        TURN_COUNTERCLOCKWISE,
        TURN_CLOCKWISE,
        STOP_MOVING,
        HitWait,
        DELAY,
        RESETONPLATFORM,
        INITIALIZEDRIVEOFFPLATFORM,
        DRIVEOFFPLATFORM,
        INITIALIZEDRIVETOWALL,
        DRIVETOWALL,
        SQUARETOWALL,
        PREPCRYPTOCOUNT,
        WAIT,
        STOPROBOT,
        ERROR_STATE
    }
    MotorState currentState = MotorState.ERROR_STATE;
    float targetHeading = 0 ;
    float magnitude = 0 ;
    float direction = 90 ;
    double delay_time = 0;
    double currentHeading = 0 ;
    double squareHeading = 0 ;
    double initialHeading = 0 ;
    double initialGyroYHeading = 0 ;
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
        waitTimer = WaitTimer.milliseconds();
        if(currentState != nextState) {
            //RobotLog.i("Change State to " + nextState);
        }
        RobotLog.i("current state is " + currentState) ;

        currentHeading = OmniBot.getcurrentHeading() ;

        currentState = nextState;

        switch(nextState) {
            case DELAY:
                current_delay = getRuntime() - last_time;
                initialHeading = currentHeading ;
                initialGyroYHeading = OmniBot.gyroScope.currentHeadingY ;
                //RobotLog.i("Delay Time " + current_delay);
                if(current_delay >= delay_time) {
                    nextState = stateAfterNext;
                }
                break;
            case WAIT_START:
                OmniBot.jewelSystem.setJewelDown();
                nextState = MotorState.DELAY;
                delay_time = 0.5;
                stateAfterNext = MotorState.CHECK_COLOR;
                last_time = getRuntime();
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
                WaitTimer.reset();
                nextState = MotorState.HitWait;
                break;
            case TURN_CLOCKWISE:
                targetHeading = (float) (currentHeading - 10.0);
                RobotLog.i("Start Turn " + currentHeading);
                OmniBot.driveOmniBot(0,0,targetHeading,PIDAxis.gyro);
                WaitTimer.reset();
                nextState = MotorState.HitWait ;
                break;
            case HitWait:
                if (WaitTimer.time() >= 5000){
                    nextState = MotorState.STOP_MOVING   ;
                } else if (Math.abs(currentHeading - targetHeading) <= 1) {
                    RobotLog.i("Reach Target Heading" + targetHeading);
                    nextState = MotorState.STOP_MOVING;
                    OmniBot.setBotMovement(0,0,0,0);
                }
                break;
            case STOP_MOVING:
                OmniBot.jewelSystem.setJewelUp();
                OmniBot.setBotMovement(0,0,0,0);
                WaitTimer.reset();
                nextState = MotorState.RESETONPLATFORM ;
                RobotLog.i("initial heading " + initialHeading) ;

                break;
            case RESETONPLATFORM:
                 targetHeading =  (float) initialHeading ;
                OmniBot.resetFirstPIDDrive();
                OmniBot.driveOmniBot(0, 0, targetHeading, PIDAxis.gyro);
                RobotLog.i("corrector is " + OmniBot.correction) ;
                if (Math.abs(currentHeading - targetHeading) <= 3) {
                    OmniBot.setBotMovement(0, 0, 0, 0);
                    nextState =MotorState.WAIT ;
                    stateAfterNext = MotorState.INITIALIZEDRIVEOFFPLATFORM ;
                }
                break;
            case INITIALIZEDRIVEOFFPLATFORM:
                OmniBot.setBotMovement(0.1, 0.1, 0.1, 0.1);
                if ( Math.abs(OmniBot.gyroScope.currentHeadingY - initialGyroYHeading) >= 4 ) {
                    RobotLog.i("initial gyro heading is " + initialGyroYHeading) ;
                    nextState = MotorState.DRIVEOFFPLATFORM ;
                }
                break;
            case DRIVEOFFPLATFORM:
                if ( Math.abs(OmniBot.gyroScope.currentHeadingY - initialGyroYHeading) <= 3 ) {
                    OmniBot.setBotMovement(0, 0, 0, 0);
                    nextState = MotorState.WAIT ;
                    WaitTimer.reset();
                    stateAfterNext = MotorState.INITIALIZEDRIVETOWALL ;
                }
                break;
            case INITIALIZEDRIVETOWALL:
                OmniBot.resetFirstPIDDrive();
                OmniBot.driveOmniBot((float) 0.2, 0, (float) initialHeading, PIDAxis.gyro);
                OmniBot.crypto.lowerCryptoServo();
                nextState = MotorState.DRIVETOWALL ;
                break;
            case DRIVETOWALL:
                OmniBot.resetFirstPIDDrive();
                OmniBot.driveOmniBot((float) 0.2, 0, (float) initialHeading, PIDAxis.gyro);
                if ( OmniBot.crypto.cryptoBoxEndTouch.getState() /*Math.abs(OmniBot.gyroScope.currentAccelerationY) >= 2 || Math.abs(OmniBot.gyroScope.currentAccelerationX) >= 2 || OmniBot.gyroScope.currentAccelerationZ >= 2*/) {
                    OmniBot.setBotMovement(0, 0, 0, 0);
                    nextState = MotorState.WAIT ;
                    WaitTimer.reset();
                    stateAfterNext = MotorState.STOPROBOT ;
                }
                break;
            case SQUARETOWALL:
                OmniBot.setBotMovement(0.1, 0.1, 0.1 , 0.1);
                if (WaitTimer.time() >= 1000) {
                    squareHeading = currentHeading ;
                    nextState = MotorState.PREPCRYPTOCOUNT ;
                }
                break;
            case PREPCRYPTOCOUNT:
                OmniBot.driveOmniBot((float) -0.1, 0, (float) squareHeading, PIDAxis.gyro);
                break;
            case WAIT:
                if (WaitTimer.time() >= 500){
                    nextState = stateAfterNext ;
                    WaitTimer.reset();
                    RobotLog.i("Robot waited for " + WaitTimer.time() + " milliseconds") ;
                }
                break;
            case STOPROBOT:
                OmniBot.setBotMovement(0, 0, 0, 0);
                RobotLog.i("Stopped bot");
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
                .addData("State ", new Func<String>() {
                    @Override
                    public String value() {
                        return String.valueOf(currentState);
                    }
                }) ;
        telemetry.addLine()
                .addData("Waittime ", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatDouble(waitTimer);
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

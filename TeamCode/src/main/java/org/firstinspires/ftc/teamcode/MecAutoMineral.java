package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Func;

import java.math.MathContext;

/**
 * Created by conno on 8/17/2017.
 */

@Autonomous(name="Mec: AutoMineral", group="Mec")
public class MecAutoMineral extends OpMode {
    public enum MotorState{
        WAIT_START,  //0
        CHECK_ROBOT_DOWN,  //1
        CHECK_HOOK_RELEASE, //2
        TURN_COUNTERCLOCKWISE,  //3
        TURN_CLOCKWISE,  //4
        STOP_MOVING,  //5
        HitWait, //6
        DELAY,  //7
        INITIALIZEDRIVEOFFPLATFORM,  //8
        DRIVEOFFPLATFORM,  //9
        DRIVETOSAFEZONE,  //10
        STOPROBOT,  //11
        WAIT,  //12
        ERROR_STATE,  //13
        RAISE_ROBOT,
        CHECK_ROBOT_UP,
        DRIVE_TO_VUFORIA
    }
    MotorState currentState = MotorState.ERROR_STATE;
    float targetHeading = 0 ;
    float magnitude = 0 ;
    float direction = 90 ;
    double delay_time = 0;
    double currentHeading = 0 ;
    robotHWconnected autoConnectedHW = robotHWconnected.MotorGyroVuforWebcam;
    HardwareRukusMecBot MecBot ;
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
        MecBot = new HardwareRukusMecBot(autoConnectedHW) ;
        MecBot.init(hardwareMap, Color.Blue);

        int count = 0;
        currentState = MotorState.WAIT_START;
        nextState = MotorState.WAIT_START;
        composeTelemetry();
    }

    @Override
    public void start() {
        MecBot.start();
    }

    @Override
    public void loop() {
        if(currentState != nextState) {
            RobotLog.i("Change State to " + nextState);
        }

        currentHeading = MecBot.getcurrentHeading() ;
        currentState = nextState;

        switch(nextState) {
            case DELAY:
                current_delay = getRuntime() - last_time;
                RobotLog.i("Delay Time " + current_delay);
                if (current_delay >= delay_time) {
                    nextState = stateAfterNext;
                }
                break;
            case WAIT_START:
                MecBot.lowerRobot();
                nextState = MotorState.CHECK_ROBOT_DOWN;
                stateAfterNext = MotorState.CHECK_ROBOT_DOWN;
                last_time = getRuntime();
                break;
            case CHECK_ROBOT_DOWN:
                if (MecBot.robotDown()) {
                    MecBot.releaseHook();
                    nextState = MotorState.CHECK_HOOK_RELEASE;
                    stateAfterNext = MotorState.CHECK_HOOK_RELEASE;
                }
                break;
            case CHECK_HOOK_RELEASE:
                if (MecBot.hookReleased()) {
                    MecBot.releaseHook();
                    nextState = MotorState.RAISE_ROBOT;
                    stateAfterNext = MotorState.CHECK_HOOK_RELEASE;
                }
                break;
            case RAISE_ROBOT:
                MecBot.raiseRobot();
                nextState = MotorState.CHECK_ROBOT_UP;
                break;
            case CHECK_ROBOT_UP:
                if (MecBot.robotUp()) {
                    MecBot.lifterStop();
                    nextState = MotorState.TURN_CLOCKWISE;
                }
                break;
            case TURN_COUNTERCLOCKWISE:
                targetHeading = (float) (currentHeading + 100.0);
                MecBot.driveBot(0,0,targetHeading,PIDAxis.gyro);
                WaitTimer.reset();
                nextState = MotorState.HitWait;
                break;
            case TURN_CLOCKWISE:
                targetHeading = (float) (currentHeading - 50.0);
                RobotLog.i("Start Turn " + currentHeading);
                MecBot.driveBot(0,0,targetHeading,PIDAxis.gyro);
                delay_time = 5;
                nextState = MotorState.DELAY ;
                stateAfterNext = MotorState.DRIVE_TO_VUFORIA;
                break;
            case DRIVE_TO_VUFORIA:
                if(MecBot.VuRukusSeen()){
                    nextState = MotorState.STOPROBOT;
                }
                break;
            case INITIALIZEDRIVEOFFPLATFORM:
                //red side (i think)
                MecBot.setBotMovement((double) -0.3, (double) -0.3, (double) -0.3, (double) -0.3);
                if ( Math.abs(MecBot.gyroScope.currentHeadingY) >= 2.5) {
                    nextState = MotorState.DRIVEOFFPLATFORM ;
                }
                break;
            case DRIVEOFFPLATFORM:
                if (Math.abs(MecBot.gyroScope.currentHeadingY) <= 2.5 ) {
                    MecBot.setBotMovement(-0.1, -0.1, -0.1, -0.1);
                    WaitTimer.reset();
                    nextState = MotorState.WAIT ;
                    stateAfterNext = MotorState.DRIVETOSAFEZONE;
                }
                break;
            case DRIVETOSAFEZONE:
                MecBot.setBotMovement(-0.1, -0.1, -0.1, -0.1);
                if (Math.abs(MecBot.gyroScope.currentAccelerationY) >= 100 || Math.abs(MecBot.gyroScope.currentAccelerationX) >= 100 || StabilizationTimer.time() >= 5000) {
                    MecBot.setBotMovement(0, 0, 0, 0);
                    nextState = MotorState.STOPROBOT ;
                }
                break;
            case STOPROBOT:
                MecBot.setBotMovement(0, 0, 0, 0);
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
                MecBot.setBotMovement(0,0,0,0);
                RobotLog.i("error no case");
                break;
        }

        telemetry.update();
        MecBot.waitForTick(40);
    }

    @Override
    public void stop() {
        MecBot.setBotMovement(0,0,0,0);
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
        MecBot.addTelemetry(telemetry);

    }
}

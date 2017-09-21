package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraDevice;

/**
 * Created by conno on 8/17/2017.
 */

@TeleOp(name="Omni: AutoOp1", group="Omni")


public class OmniAutoOp2 extends OpMode {
    public enum MotorState{
        Drive,
        Turn,
        InitializeTurn,
        WAIT_START_PERIOD,
        WAIT_PERIOD,
        WAIT_DRIVE_FORWARD,
        STOP_MOVING,
        DONE,
        DRIVE_FORWARD_TO_BALL,
        ERROR_STATE
    }
    MotorState currentState = MotorState.ERROR_STATE;
    float motorLeft1power = 0;
    float motorLeft2power = 0;
    float motorRight1power = 0;
    float motorRight2power = 0;
    float leftStickY = 0 ;
    boolean controller1 = true;
    boolean controller2 = false ;
    robotHWconnected autoConnectedHW = robotHWconnected.MotorGyro;
    HardwareOmniBot OmniBot ;
    ElapsedTime StabilizationTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    MotorState nextState = MotorState.ERROR_STATE;

    @Override
    public void init() {
        OmniBot = new HardwareOmniBot(autoConnectedHW) ;
        OmniBot.init(hardwareMap);

        int count = 0;
        currentState = MotorState.WAIT_START_PERIOD;
        nextState = MotorState.WAIT_START_PERIOD;
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

        currentState = nextState;
        switch(nextState) {
            case WAIT_START_PERIOD:
                StabilizationTimer.startTime();
                nextState = MotorState.WAIT_PERIOD;
                break;
            case WAIT_PERIOD:
                if (StabilizationTimer.time() > 500) {
                    nextState = MotorState.WAIT_DRIVE_FORWARD;
                }
                break;
            case WAIT_DRIVE_FORWARD:
                OmniBot.gyroDriveStaight(0.5, 45.0);
                if(StabilizationTimer.seconds() > 1000) {
                    nextState = MotorState.STOP_MOVING;
                } else {
                    nextState = MotorState.DRIVE_FORWARD_TO_BALL;
                }
                nextState = MotorState.ERROR_STATE;
                break;
            case STOP_MOVING:
                OmniBot.resetFirstPIDDrive();
                nextState = MotorState.DONE;
                break;
            case ERROR_STATE:
                break;
            default:
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

package org.firstinspires.ftc.teamcode;

import android.test.MoreAsserts;

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
    float motorLeft1power = 0;
    float motorLeft2power = 0;
    float motorRight1power = 0;
    float motorRight2power = 0;
    float leftStickY = 0 ;
    boolean controller1 = true;
    boolean controller2 = false ;
    robotHWconnected autoConnectedHW = robotHWconnected.MotorGyro;
    MotorState currentState = MotorState.ERROR_STATE;
    MotorState nextState = MotorState.ERROR_STATE;
    HardwareOmniBot OmniBot ;
    ElapsedTime StabilizationTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void init() {
        OmniBot = new HardwareOmniBot(autoConnectedHW) ;
        OmniBot.init(hardwareMap);

        currentState = MotorState.WAIT_TO_START;
        nextState = MotorState.WAIT_TO_START;
        int count = 0;

        composeTelemetry();
    }

    @Override
    public void start() {

        OmniBot.start();
    }

    @Override
    public void loop() {
<<<<<<< HEAD
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
=======
        leftStickY = -gamepad1.left_stick_y ;

        if (gamepad1.a || controller1) {
            controller1 = true ;
            controller2 = false ;

            OmniBot.driveOmniBot(0, 0, 0);

        }
        if (gamepad2.a || controller2) {
            controller2 = true;
            controller1 = false;
>>>>>>> 95a44fefe0e611353b6c14bd5fecf3ba576f7098

            OmniBot.driveOmniBot((float) 0, 0, 0);
        }
        OmniBot.waitForTick(40);
        telemetry.update();

    }

    @Override
    public void stop() {

    }
    void composeTelemetry() {
<<<<<<< HEAD

        OmniBot.addTelemetry(telemetry);
=======
        OmniBot.addTelemetry(telemetry);
        //OmniBot.getTelemetry(telemetry);
>>>>>>> 95a44fefe0e611353b6c14bd5fecf3ba576f7098
    }




}

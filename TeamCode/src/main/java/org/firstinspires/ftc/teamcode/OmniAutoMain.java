package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by conno on 8/17/2017.
 */

@TeleOp(name="Omni: Auto Main", group="Omni")


public class OmniAutoMain extends OpMode {
    public enum MotorState{
        INITIALIZE,
        INITIALIZEDRIVE,
        DRIVE,
        SQUAREVUFORIA,
        PUSHBALLS,
        SENSELINE,
        COUNTBARS,
        ALIGNTOBARS,
        PUSHINCUBE
    }
    MotorState currentState = MotorState.INITIALIZE;
    float magnitude = 0 ;
    float direction = 0 ;
    float targetHeading = 0 ;
    MotorState nextState = null ;
    MotorState stateAfterNext = null ;
    robotHWconnected autoConnectedHW = robotHWconnected.MotorGyro;
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

        currentState = nextState ;
        switch (currentState) {
            case INITIALIZE:
                break;
            case INITIALIZEDRIVE:
                break;
            case DRIVE:
                break;
            case SQUAREVUFORIA:
                break;
        }

    }

    @Override
    public void stop() {

    }
    void composeTelemetry() {

        OmniBot.addTelemetry(telemetry);
        //OmniBot.getTelemetry(telemetry);
    }




}

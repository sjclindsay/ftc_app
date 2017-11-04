package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by conno on 8/17/2017.
 */

@TeleOp(name="Omni: Auto Main", group="Omni")


public class OmniAutoMain extends OpMode {
    public enum MotorState{
        INITIALIZE,
        PUSHBALLS,
        INITIALIZEDRIVEOFFPLATFORM,
        DRIVEOFFPLATFORM,
        SQUAREVUFORIARX,
        SQUAREVUFORIATX,
        SQUAREVUFORIATZ,
        INITIALIZEDRIVETOCRYPTO,
        DRIVETOCRYPTO,
        COUNTBARS,
        COUNTSTAGE,
        ALIGNTOBARS,
        PUSHINCUBE,
        WAIT
    }
    MotorState currentState = MotorState.INITIALIZE;
    float magnitude = 0 ;
    float direction = 0 ;
    float targetHeading = 0 ;
    int barCounter = 0 ;
    public RelicRecoveryVuMark vuMark = null;

    MotorState nextState = null ;
    MotorState stateAfterNext = null ;
    robotHWconnected autoConnectedHW = robotHWconnected.MotorLifterVufor;
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

        telemetry.update();

    }

    @Override
    public void loop() {

        //OmniBot.driveOmniBot(0, 0, 0, PIDAxis.rx);

        OmniBot.VuReader.updateVuforiaCoords();
        /*
        currentState = nextState ;
        switch (currentState) {
            case INITIALIZE:
                if (StabilizationTimer.time() >= 500) {
                    nextState = MotorState.INITIALIZEDRIVEOFFPLATFORM ;
                }
                break;
            case INITIALIZEDRIVEOFFPLATFORM:
                //red side (i think)
                OmniBot.driveOmniBot( (float) 0.5, -90, 0, PIDAxis.gyro);
                if ( Math.abs(OmniBot.gyroScope.currentHeadingZ) >= 2.5) {
                    nextState = MotorState.DRIVEOFFPLATFORM ;
                }
                break;
            case DRIVEOFFPLATFORM:
                if (Math.abs(OmniBot.gyroScope.currentHeadingZ) <= 2.5 ) {
                    OmniBot.driveOmniBot(0, 0, 0, PIDAxis.gyro);
                    nextState = MotorState.WAIT ;
                    stateAfterNext = MotorState.SQUAREVUFORIARX ;
                }
                break;
            case SQUAREVUFORIARX:
                OmniBot.driveOmniBot(0, 0, 0, PIDAxis.rx);
                if (Math.abs(OmniBot.vufor.getVuforiaCoords(HardwareVuforia.vuForiaCoord.rX)) <= 2) {
                    OmniBot.driveOmniBot(0, 0, 0, PIDAxis.rx ) ;
                    nextState = MotorState.WAIT ;
                    stateAfterNext = MotorState.SQUAREVUFORIATX ;
                }
                break;
            case SQUAREVUFORIATX:
                OmniBot.driveOmniBot(0, 0, 0, PIDAxis.tx);
                if (Math.abs(OmniBot.vufor.getVuforiaCoords(HardwareVuforia.vuForiaCoord.tX)) <= 2) {
                    OmniBot.driveOmniBot(0, 0, 0, PIDAxis.tx ) ;
                    nextState = MotorState.WAIT ;
                    stateAfterNext = MotorState.SQUAREVUFORIATZ ;
                }
                break;
            case SQUAREVUFORIATZ:
                OmniBot.driveOmniBot(0, 0, 0, PIDAxis.tz);
                if (Math.abs(OmniBot.vufor.getVuforiaCoords(HardwareVuforia.vuForiaCoord.tZ)) <= 2) {
                    OmniBot.driveOmniBot(0, 0, 0, PIDAxis.tz ) ;
                    nextState = MotorState.WAIT ;
                    stateAfterNext = MotorState.DRIVETOCRYPTO;
                }
                break;
            case INITIALIZEDRIVETOCRYPTO:
                OmniBot.driveOmniBot( (float) 0.5, 0, 0, PIDAxis.gyro);
                nextState = MotorState.DRIVETOCRYPTO ;
                break;
            case DRIVETOCRYPTO:
                if (StabilizationTimer.time() >= 500) {
                    nextState = MotorState.WAIT ;
                    stateAfterNext = MotorState.COUNTBARS ;
                }
                break;
            case COUNTBARS:
                OmniBot.driveOmniBot( (float) 0.3, -90, 0, PIDAxis.gyro);
                OmniBot.crypto.lowerCryptoServo();
                if (OmniBot.crypto.cryptoBoxTouchValue2) {
                    barCounter += 1 ;
                    nextState = MotorState.COUNTSTAGE ;
                }
                break;
            case WAIT:
                if (StabilizationTimer.time() >= 300) {
                    nextState = stateAfterNext ;
                }
                break;
        }
        */

        vuMark =  OmniBot. VuReader.GetLocation() ;
        telemetry.addData("VuMark", "%s visible", vuMark);

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

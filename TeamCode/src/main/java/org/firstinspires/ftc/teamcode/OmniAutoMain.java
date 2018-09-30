package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by conno on 8/17/2017.
 */

@TeleOp(name="Omni: Auto Main", group="Omni")
@Disabled

public class OmniAutoMain extends OpMode {
    public enum MotorState{
        INITIALIZE,
        PUSHBALLS,
        INITIALIZEDRIVEOFFPLATFORM,
        DRIVEOFFPLATFORM,
        WAITFORVUREADER,
        SQUAREVUFORIARY,
        SQUAREVUFORIATY,
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
    robotHWconnected autoConnectedHW = robotHWconnected.MotorGyroLifterVuforLocal;
    HardwareOmniBot OmniBot ;
    ElapsedTime StabilizationTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void init() {
        OmniBot = new HardwareOmniBot(autoConnectedHW) ;
        OmniBot.init(hardwareMap, HardwareColorSensor.Color.Red);

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
        OmniBot.VuReader.updateVuforiaCoords();
        OmniBot.gyroScope.Update();
        //OmniBot.driveOmniBot(0, 0, 0, PIDAxis.ry);

        RobotLog.i("Stabilization timer is " + StabilizationTimer.time()) ;
        RobotLog.i("state is " + currentState) ;

        switch (currentState) {
            case INITIALIZE:
                if (StabilizationTimer.time() >= 500) {
                    nextState = MotorState.INITIALIZEDRIVEOFFPLATFORM ;
                }
                break;
            case INITIALIZEDRIVEOFFPLATFORM:
                //red side (i think)
                OmniBot.driveOmniBot( (float) 0.3, 0, 0, PIDAxis.gyro);
                if ( Math.abs(OmniBot.gyroScope.currentHeadingY) >= 2.5) {
                    nextState = MotorState.DRIVEOFFPLATFORM ;
                }
                break;
            case DRIVEOFFPLATFORM:
                if (Math.abs(OmniBot.gyroScope.currentHeadingY) <= 2.5 ) {
                    OmniBot.driveOmniBot(0, 0, 0, PIDAxis.gyro);
                    nextState = MotorState.WAIT ;
                    StabilizationTimer.reset();
                    stateAfterNext = MotorState.WAITFORVUREADER ;
                }
                break;
            case WAITFORVUREADER:
                if (OmniBot.VuReader.getVuforiaCoords(HardwareVuforia.vuForiaCoord.tZ) == 0 || StabilizationTimer.time() <= 2000) {

                } else if (OmniBot.VuReader.getVuforiaCoords(HardwareVuforia.vuForiaCoord.tZ) != 0 ) {
                    nextState = MotorState.SQUAREVUFORIARY ;
                } else {
                    nextState = MotorState.INITIALIZEDRIVETOCRYPTO ;
                }
                break;
            case SQUAREVUFORIARY:
                OmniBot.driveOmniBot(0, 0, 0, PIDAxis.ry);
                RobotLog.i("RY correction is " + OmniBot.correction) ;
                if (Math.abs(OmniBot.VuReader.getVuforiaCoords(HardwareVuforia.vuForiaCoord.rY)) <= 2) {
                    OmniBot.driveOmniBot(0, 0, 0, PIDAxis.ry ) ;
                    nextState = MotorState.WAIT ;
                    StabilizationTimer.reset();
                    stateAfterNext = MotorState.SQUAREVUFORIATY ;
                }
                break;
            case SQUAREVUFORIATY:
                OmniBot.driveOmniBot(0, 0, 0, PIDAxis.ty);
                if (Math.abs(OmniBot.VuReader.getVuforiaCoords(HardwareVuforia.vuForiaCoord.tY)) <= 2) {
                    OmniBot.driveOmniBot(0, 0, 0, PIDAxis.ty ) ;
                    nextState = MotorState.WAIT ;
                    StabilizationTimer.reset();
                    stateAfterNext = MotorState.SQUAREVUFORIATZ ;
                }
                break;
            case SQUAREVUFORIATZ:
                OmniBot.driveOmniBot(0, 0, 0, PIDAxis.tz);
                if (500 - Math.abs(OmniBot.VuReader.getVuforiaCoords(HardwareVuforia.vuForiaCoord.tZ)) <= 2) {
                    OmniBot.driveOmniBot(0, 0, 0, PIDAxis.tz ) ;
                    nextState = MotorState.WAIT ;
                    StabilizationTimer.reset();
                    stateAfterNext = null ;
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
                    StabilizationTimer.reset();
                }
                break;
        }

        currentState = nextState ;

        vuMark =  OmniBot.VuReader.GetLocation() ;
        telemetry.update();

    }

    @Override
    public void stop() {

    }
    void composeTelemetry() {

        OmniBot.addTelemetry(telemetry);

    }




}
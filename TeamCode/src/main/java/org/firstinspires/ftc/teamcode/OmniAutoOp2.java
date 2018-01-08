package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraDevice;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by conno on 8/17/2017.
 */

@TeleOp(name="Omni: AutoOp2", group="Omni")
@Disabled

public class OmniAutoOp2 extends OpMode {
    public enum MotorState{
        Drive,
        Turn,
        InitializeTurn,
        WAIT_START_PERIOD,
        WAIT_PERIOD,
        WAIT_DRIVE_FORWARD,
        STOP_MOVING,
        ERROR_STATE
    }
    MotorState currentState = MotorState.ERROR_STATE;
    float targetHeading = 0 ;
    float magnitude = 0 ;
    float direction = 90 ;
    double currentHeading = 0 ;
    robotHWconnected autoConnectedHW = robotHWconnected.MotorGyroLifterVufor;
    HardwareOmniBot OmniBot ;
    ElapsedTime StabilizationTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime WaitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double waitTimer  = 0 ;
    MotorState nextState = MotorState.WAIT_START_PERIOD;
    MotorState stateAfterNext = MotorState.InitializeTurn ;

    @Override
    public void init() {
        OmniBot = new HardwareOmniBot(autoConnectedHW) ;
        OmniBot.init(hardwareMap, HardwareColorSensor.Color.Red);

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

        OmniBot.driveOmniBot(0,0,0,PIDAxis.ry);
        /*if(currentState != nextState) {
            RobotLog.i("Change State to " + nextState);
        }

        waitTimer = WaitTimer.time() ;
        currentHeading = OmniBot.getcurrentHeading() ;

        currentState = nextState;

        switch(nextState) {
            case WAIT_START_PERIOD:
                StabilizationTimer.startTime();
                nextState = MotorState.WAIT_PERIOD;

                break;
            case WAIT_PERIOD:
                if (StabilizationTimer.time() > 500) {
                    nextState = stateAfterNext ;
                }
                break;
            case InitializeTurn:
                OmniBot.resetFirstPIDDrive();
                targetHeading = (float) currentHeading + 90 ;
                OmniBot.driveOmniBot(0, 0, targetHeading, PIDAxis.gyro);
                nextState = MotorState.Turn ;
                RobotLog.i("start turn") ;
                break;
            case Turn:
                magnitude = 0 ;
                direction = 90 ;
                if (targetHeading - currentHeading <= 2 || targetHeading - currentHeading >= -2) {
                    nextState = MotorState.Drive ;
                    OmniBot.resetFirstPIDDrive();
                    WaitTimer.reset();
                    RobotLog.i("finish turn") ;
                }
                break;
            case Drive:
                OmniBot.driveOmniBot((float) 0.1, 0, targetHeading, PIDAxis.gyro);
                if (waitTimer >= 1000) {
                    nextState = MotorState.WAIT_PERIOD ;
                    stateAfterNext = MotorState.InitializeTurn ;
                    RobotLog.i("finish drive") ;
                }
            case STOP_MOVING:
                OmniBot.driveOmniBot(0, 0, (float)currentHeading, PIDAxis.gyro);
                break;
            default:
                OmniBot.driveOmniBot(0, 0, (float)currentHeading, PIDAxis.gyro);
                RobotLog.i("error no case");
                break;
        }
*/

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

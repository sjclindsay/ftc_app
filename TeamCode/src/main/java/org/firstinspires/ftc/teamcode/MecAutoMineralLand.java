package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Func;

/**
 * Created by conno on 8/17/2017.
 */

@Autonomous(name="Mec: AutoMineralLand", group="Mec")
public class MecAutoMineralLand extends OpMode {
    public enum MotorState{
        WAIT_START,  //0
        CHECK_ROBOT_DOWN,  //1
        CHECK_HOOK_RELEASE, //2
        TURN_COUNTERCLOCKWISE,  //3
        TURN_CLOCKWISE,
        WAIT_TURN_COMPLETE,//4
        STOP_MOVING,  //5
        HitWait, //6
        DELAY,  //7
        INITIALIZEDRIVEOFFPLATFORM,  //8
        DRIVEOFFPLATFORM,  //9
        STOPROBOT,  //11
        WAIT,  //12
        ERROR_STATE,  //13
        RAISE_ROBOT,
        CHECK_ROBOT_UP,
        DRIVE_TO_VUFORIA,
        STOP_TO_VUFORIA,
        DRIVE_TO_X,
        TURN_COUNTERCLOCKWISE_VU,
        DRIVE_TO_WALL,
        WAIT_DRIVE_TO_WALL,
        DRIVE_AWAY_FROM_WALL,
        DRIVE_TO_SAFE_ZONE,
        HIT_WALL,
        DRIVE_TO_RELEASE_POINT,
        DELIVER_PAYLOAD,
        SPIN_TO_CRATER,
        RETURN_TO_CRATER,
        BACK_AWAY_FROM_HILL,
        SQUARE_TO_HILL,
        CLIMB_HILL



    }
    MotorState currentState = MotorState.ERROR_STATE;
    float targetHeading = 0 ;
    float targetHeadingY = 0 ;
    float magnitude = 0 ;
    float direction = 90 ;
    float target_mag = 0;
    float target_dir = 0;
    double delay_time = 0;
    double target_x = 200 ;
    double currentHeading = 0 ;
    double kp = 0.0055 ;
    double ki = 0.000001 ;
    robotHWconnected autoConnectedHW = robotHWconnected.MotorGyroVuforWebcamLifterMarkerGrabber;
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
                MecBot.raiseRobot();
                nextState = MotorState.CHECK_ROBOT_DOWN;
                stateAfterNext = MotorState.CHECK_ROBOT_DOWN;
                last_time = getRuntime();
                break;
            case CHECK_ROBOT_DOWN:
                if (MecBot.robotDown()) {
                    nextState = MotorState.CHECK_HOOK_RELEASE;
                    stateAfterNext = MotorState.CHECK_HOOK_RELEASE;
                    MecBot.lifterStop();
                }
                break;
            case CHECK_HOOK_RELEASE:
                RobotLog.i("Start Turn " + currentHeading);
                MecBot.resetFirstPIDDrive(0.0055,0.000001);
                MecBot.driveBot((float)0.2,-90,(float) currentHeading,PIDAxis.gyro);
                delay_time = 3;
                nextState = MotorState.DELAY;
                stateAfterNext = MotorState.TURN_CLOCKWISE;
                break;
            case RAISE_ROBOT:
                MecBot.setBotMovement(0,0,0,0);
                MecBot.lowerRobot();
                nextState = MotorState.STOPROBOT;
                break;
            case CHECK_ROBOT_UP:
                if (MecBot.robotUp()) {
                    MecBot.lifterStop();
                    nextState = MotorState.STOPROBOT;
                }
                break;
            case TURN_COUNTERCLOCKWISE:
                targetHeading = (float) (currentHeading + 100.0);
                MecBot.resetFirstPIDDrive(0.0055, 0.000001);
                MecBot.driveBot(0,0,targetHeading,PIDAxis.gyro);
                WaitTimer.reset();
                nextState = MotorState.HitWait;
                break;
            case TURN_CLOCKWISE:
                targetHeading = (float) (currentHeading - 50.0);
                RobotLog.i("Start Turn " + currentHeading);
                MecBot.resetFirstPIDDrive(0.0055,0.000003);
                MecBot.driveBot(0,0,targetHeading,PIDAxis.gyro);
                target_mag = 0;
                target_dir= 0;
                nextState = MotorState.WAIT_TURN_COMPLETE ;
                stateAfterNext = MotorState.STOPROBOT;
                MecBot.resetFirstPIDDrive(0.0055, 0.000003);
                break;
            case WAIT_TURN_COMPLETE:
                MecBot.driveBot(target_mag,target_dir, targetHeading,PIDAxis.gyro);
                if(Math.abs(targetHeading - currentHeading) <= 2){
                    nextState=stateAfterNext;
                    WaitTimer.reset();
                    MecBot.setBotMovement(0,0,0,0);
                }
                break;
            case DRIVE_TO_VUFORIA:
                MecBot.driveBot((float) -0.15,0,targetHeading,PIDAxis.gyro);
                if(MecBot.VuRukusSeen()){
                    targetHeading = targetHeading+10 ;
                    target_x = 0.0 ;
                    MecBot.resetFirstPIDDrive(0.0055, 0.00001);
                    nextState = MotorState.TURN_COUNTERCLOCKWISE_VU ;
                } else if (WaitTimer.milliseconds() >= 1250) {
                    WaitTimer.reset();
                    nextState = MotorState.STOP_TO_VUFORIA;
                }
                break;
            case STOP_TO_VUFORIA:
                MecBot.setBotMovement(0,0,0,0);
                if(MecBot.VuRukusSeen()){
                    targetHeading = targetHeading+5 ;
                    target_x = 0.0 ;
                    MecBot.resetFirstPIDDrive(0.0055,0.00001);
                    nextState = MotorState.TURN_COUNTERCLOCKWISE_VU;
                } else if (WaitTimer.milliseconds() >= 750) {
                    WaitTimer.reset();
                    nextState = MotorState.DRIVE_TO_VUFORIA ;
                }
                break;
            case TURN_COUNTERCLOCKWISE_VU:
                MecBot.driveBot((float) 0, 0, targetHeading, PIDAxis.gyro);
                if (Math.abs(targetHeading-currentHeading) <= 1 ) {
                    nextState = MotorState.DRIVE_TO_WALL;
                    MecBot.resetFirstPIDDrive(0.0055, 0.000002);
                    StabilizationTimer.reset();

                }
                break;
                /*
            case DRIVE_TO_X:
                MecBot.driveBot((float)-0.15,0,(float)target_x,PIDAxis.tx);
                if (Math.abs(MecBot.getVuX()- target_x) <= 2){
                    MecBot.setBotMovement(0,0,0,0);
                    nextState = MotorState.DRIVE_TO_WALL ;
                    StabilizationTimer.reset();
                    MecBot.resetFirstPIDDrive(0.00055,0.000002);
                } else if (MecBot.VuRukusSeen()== false){
                    MecBot.setBotMovement(0,0,0,0);
                    nextState = MotorState.STOPROBOT ;
                }
                break;
                */
            case DRIVE_TO_WALL:
                MecBot.driveBot((float) -0.4, 0,targetHeading, PIDAxis.gyro);
                if (StabilizationTimer.time() > 1000) {
                    nextState = MotorState.WAIT_DRIVE_TO_WALL ;
                    WaitTimer.reset();
                }
                break;
            case WAIT_DRIVE_TO_WALL:
                MecBot.driveBot((float) -0.4, 0,targetHeading, PIDAxis.gyro);
                if (Math.abs(MecBot.getCurrentAccelerationX()) > 4) {
                    MecBot.setBotMovement(-0.4 ,-0.4, -0.4, -0.4);
                    StabilizationTimer.reset();
                    nextState = MotorState.WAIT ;
                    stateAfterNext = MotorState.DRIVE_TO_SAFE_ZONE;
                    targetHeading = (float) currentHeading ;
                } else if (WaitTimer.time() > 4000){
                    nextState = MotorState.STOPROBOT ;
                }
                break;
            case DRIVE_AWAY_FROM_WALL:
                targetHeading = (float) currentHeading ;
                MecBot.setBotMovement(0.4,0.4,0.4,0.4);
                WaitTimer.reset();
                nextState = MotorState.WAIT ;
                stateAfterNext = MotorState.DRIVE_TO_SAFE_ZONE ;
                MecBot.resetFirstPIDDrive(0.0055,0.000001);
                break;
            case DRIVE_TO_SAFE_ZONE:
                MecBot.driveBot((float) 0.4, -90, targetHeading, PIDAxis.gyro);
                if (StabilizationTimer.time() > 1000) {
                    nextState = MotorState.HIT_WALL ;
                    StabilizationTimer.reset();
                }
                break;
            case HIT_WALL:
                MecBot.driveBot( (float) 0.6, -90, targetHeading, PIDAxis.gyro );
                if (Math.abs(MecBot.getCurrentAccelerationY()) > 3.5) {
                    MecBot.setBotMovement(0, 0, 0, 0);
                    nextState = MotorState.DRIVE_TO_RELEASE_POINT ;
                    MecBot.resetFirstPIDDrive(0.0055,0.000001);
                    WaitTimer.reset();
                } else if (StabilizationTimer.time() > 3000) {
                    MecBot.setBotMovement(0, 0, 0, 0);
                    nextState = MotorState.DRIVE_TO_RELEASE_POINT ;
                    MecBot.resetFirstPIDDrive(0.0055,0.000001);
                    WaitTimer.reset();
                }
                break;
            case DRIVE_TO_RELEASE_POINT:
                MecBot.driveBot( (float) 0.4, 90, targetHeading, PIDAxis.gyro);
                if (WaitTimer.time() > 2500) {
                    nextState = MotorState.DELIVER_PAYLOAD ;
                    MecBot.setBotMovement(0, 0, 0, 0);
                    WaitTimer.reset();
                }
                break;
            case DELIVER_PAYLOAD:
                MecBot.dropmarker();
                if (WaitTimer.time() > 500) {
                    MecBot.liftmarker();
                    nextState = MotorState.SPIN_TO_CRATER ;
                    targetHeadingY = MecBot.gyroScope.currentHeadingY ;
                    targetHeading = targetHeading + 89 ;
                    MecBot.resetFirstPIDDrive(kp, 0.000002);
                }
                break;
            case SPIN_TO_CRATER:
                MecBot.driveBot(0, 0, targetHeading, PIDAxis.gyro);
                if (Math.abs(targetHeading - currentHeading) < 1) {
                    nextState = MotorState.RETURN_TO_CRATER ;
                    MecBot.setBotMovement(0, 0, 0, 0);
                }
                break;
            case RETURN_TO_CRATER:
                //dont fail
                MecBot.driveBot( (float) 0.7, 0, targetHeading, PIDAxis.gyro);
                if (Math.abs(targetHeadingY - MecBot.gyroScope.currentHeadingY) > 4) {
                    nextState = MotorState.CLIMB_HILL;
                }
                break;
            case BACK_AWAY_FROM_HILL:
                MecBot.driveBot((float)0.15, -90, targetHeading, PIDAxis.gyro);
                if (Math.abs(targetHeadingY - MecBot.gyroScope.currentHeadingY) < 2) {
                    nextState = MotorState.SQUARE_TO_HILL ;
                    targetHeading += 90 ;
                }
                break;
            case SQUARE_TO_HILL:
                MecBot.driveBot((float) 0.15, 0, targetHeading, PIDAxis.gyro);
                if (Math.abs(targetHeading - MecBot.gyroScope.currentHeadingX) < 3) {
                    WaitTimer.reset();
                    nextState = MotorState.WAIT ;
                    stateAfterNext = MotorState.CLIMB_HILL ;
                    MecBot.resetFirstPIDDrive(kp, ki);
                    MecBot.driveBot((float) 0.04, 0, targetHeading, PIDAxis.gyro);
                }
                break;
            case CLIMB_HILL:
                MecBot. driveBot((float) 0.4, 0, targetHeading, PIDAxis.gyro);
                if (Math.abs(targetHeadingY - MecBot.gyroScope.currentHeadingY) < 2) {
                    MecBot.setBotMovement(0, 0, 0, 0);
                    nextState = MotorState.STOPROBOT ;
                    RobotLog.i("Finished program!!!!!") ;
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

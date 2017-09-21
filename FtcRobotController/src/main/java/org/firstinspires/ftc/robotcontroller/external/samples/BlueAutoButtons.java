/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.ar.pl.DebugLog;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@Autonomous(name="Blue: BlueAutoButtons", group="WeCo")
// @Disabled
public class BlueAutoButtons extends OpMode  {
    public enum MotorState{
        ERROR_STATE,
        WAIT_TO_START,
        WAIT_PERIOD,
        WAIT_START_PERIOD,
        WAIT_FOR_STABLE,
        DRIVE_FORWARD,
        TURN_LEFT,
        TURN_LEFT_BACK,
        TURN_RIGHT,
        TURN_RIGHT_BACK,
        BACK_UP_TO_TURN,
        BACK_UP_SENSE_BUTTON,
        BACK_UP_TO_VORTEX,
        WAIT_DRIVE_VORTEX,
        WAIT_DRIVE_FORWARD,
        WALL_FOLLOW,
        WALL_FOLLOW_BACK,
        WALL_FOLLOW_TAPE,
        WALL_FOLLOW_BACK_TOUCH,
        WALL_FOLLOW_PASSED_TAPE,
        WALL_FOLLOW_PAST_TAPE_BACK,
        SENSE_BUTTON,
        PUSH_FRONT_BUTTON,
        PUSH_BACK_BUTTON,
        ARE_WE_DONE,
        NEXT_BUTTON,
        WAIT,
        STOP_ROBOT,
        DONE
    }

    public enum SensorColor{
        Red,
        Blue
    }
    static double startWaitPeriod = 0.0;
    //Drive Control Values
    static final float normalTurnSpeed = (float) 0.1;
    private float parkSpeedDelta = (float) 0.03;
    static final float normalSpeed = (float) 0.13 ;
    static final float sideSpeed = (float) 0.6;
    private int leftTurnCount = 0;
    static final float normalLine = 1;
    static final double normal90turn = 60;
    static final double WheelPositionDivisior = 2500.0;
    static final double ACCLERATION_STABILITY = 0.1; //This is in m/s^2
    private MotorState currentState;
    private MotorState nextState;
    private MotorState nextStateAfterWait;
    // Initialize HW Data Objects
    private Servo servoLeftRight;
    private Servo servoPushButton;
    private Servo servoUpDown;
    private Servo servoTapeLeft;
    private Servo servoTapeRight;
    private static final int LED_CHANNEL = 0;
    DeviceInterfaceModule cdim;

    private ColorSensor sensorRGB;

    private static double SERVOLEFTRIGHT_STARTPOSITION = 0.5;
    private static double SERVOUPDOWN_STARTPOSITION = 0.2;
    private static double SERVOPUSHBUTTON_STARTPOSITION = 0.1;


    TouchSensor touchSensor3;  // Hardware Device Object
    TouchSensor touchSensor2;  // Hardware Device Object
    LightSensor lightSensor0;  // Hardware Device Object
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    Acceleration prevGravity;

    DcMotor motorLeft1;
    DcMotor motorLeft2;
    DcMotor motorRight1;
    DcMotor motorRight2;
    DcMotor motorSideButton;
    Servo servoSideButtonPush ;

    int resetValueLeft = 0;
    int resetValueRight = 0;
    double resetValueHeading = 0;
    double degreesTurned = 0;
    float motorPowerMin = -1;
    float motorPowerMax = 1;
    double positionLeft = 0;
    double positionRight = 0;
    double sideButtonPushPosition ;
    float SideCurrentPosition = 0;
    float motorLeftPower = 0;
    float motorRightPower = 0;
    float motorSideButtonPower = 0;
    double driveCorrection = 0.0;
    PIDController motorPID;
    Orientation RobotAngles;
    double currentHeading = 0.0;
    double pitch = 0.0 ;
    float startOrientation;
    ElapsedTime StabilizationTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS) ;

    private double TargetHeading;
    double buttonPressCount ;

    public BlueAutoButtons() {
    }

    @Override
    public void init() {

        // get a reference to our Hardware objects
        touchSensor2 = hardwareMap.touchSensor.get("touchSensorP2");
        touchSensor3 = hardwareMap.touchSensor.get("touchSensorP3");
        lightSensor0 = hardwareMap.lightSensor.get("lightSensorP0");
        servoPushButton = hardwareMap.servo.get("servoButtonP3");
        servoLeftRight = hardwareMap.servo.get("servoLeftRightP1");
        servoUpDown = hardwareMap.servo.get("servoUpDownP2");
        servoSideButtonPush = hardwareMap.servo.get("servoSideButtonPush") ;


// get a reference to our DeviceInterfaceModule object.
        cdim = hardwareMap.deviceInterfaceModule.get("dim");

        // set the digital channel to output mode.
        // remember, the Adafruit sensor is actually two devices.
        // It's an I2C sensor and it's also an LED that can be turned on or off.
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        sensorRGB = hardwareMap.colorSensor.get("sensor_color");

        motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight1 = hardwareMap.dcMotor.get("motorRight1");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");
        motorSideButton = hardwareMap.dcMotor.get("motorSideButton");

        //Setup Hardware
        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);


        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        currentState = MotorState.WAIT_TO_START;
        nextState = MotorState.WAIT_TO_START;
        count = 0;
        buttonPressCount = 0;
        composeTelemetry();
    }
    //Event Control Value
    double count;

    double etime;

    @Override
    public void start() {
        lightSensor0.enableLed(true);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 200);
        //setup gimble
        servoPushButton.setPosition(SERVOPUSHBUTTON_STARTPOSITION);
        servoLeftRight.setPosition(SERVOLEFTRIGHT_STARTPOSITION);
        servoUpDown.setPosition(SERVOUPDOWN_STARTPOSITION);

        servoSideButtonPush.setPosition(0.5);
        cdim.setDigitalChannelState(LED_CHANNEL,false);
    }

    @Override
    public void loop() {

        telemetry.update();
        if(currentState != nextState) {
            RobotLog.i("Current State is " + nextState);
        }
        currentState = nextState;
        pitch = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.thirdAngle)) ;
        //RobotAngles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        currentHeading = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle));
        sideButtonPushPosition = motorSideButton.getCurrentPosition() ;
        switch(currentState) {
            case WAIT_START_PERIOD:
                StabilizationTimer.startTime();
                nextState = MotorState.WAIT_PERIOD;
                break;
            case WAIT_PERIOD:
                if(StabilizationTimer.time() > 500) {
                    nextState = MotorState.WAIT_DRIVE_FORWARD;
                }
                break;
            case WAIT_TO_START:
                nextState = MotorState.DRIVE_FORWARD;
                break;
            case DRIVE_FORWARD: //Not really anymore :) this turns 40-45 degress. trying something better than trying to align on the wall.
                // Now just put bck of bot flat on the wall.
                resetValueLeft = -motorLeft1.getCurrentPosition();
                resetValueRight = motorRight1.getCurrentPosition();
                TargetHeading = currentHeading -40;
                MoveForward(TargetHeading);
                nextState = MotorState.WAIT_DRIVE_FORWARD;
                break;
            case WAIT_DRIVE_FORWARD:
                driveCorrection = StabilizeMeOnCurrentHeading(currentHeading,TargetHeading);
                motorLeftPower = normalSpeed - (float) driveCorrection;
                motorRightPower = normalSpeed + (float) driveCorrection;
                if(touchSensor2.isPressed()) {
                    nextState = MotorState.BACK_UP_TO_TURN;
                    waitTimer.reset();
                }
                break;
            case BACK_UP_TO_TURN:
                StartPivotLeft();
                //BackUp();
                if (touchSensor3.isPressed()) {
                    nextState = MotorState.TURN_LEFT_BACK ;
                        waitTimer.reset();
                        TargetHeading = currentHeading;
                }
                break;
            case TURN_LEFT_BACK:
                TurnLeftBack();
                if(!touchSensor3.isPressed() || waitTimer.time() > 1000 )
                    nextState = MotorState.WALL_FOLLOW ;
                break;
            case TURN_RIGHT:
                if(touchSensor3.isPressed() && touchSensor2.isPressed()) {
                    //if(currentHeading - TargetHeading >= 6) {
                    nextState = MotorState.BACK_UP_SENSE_BUTTON;
                    StopMove();
                } else if(touchSensor2.isPressed()){
                    nextState = MotorState.TURN_LEFT;
                    waitTimer.reset();
                }
                break;
            case TURN_RIGHT_BACK:

                if(touchSensor3.isPressed() && touchSensor2.isPressed()) {
                    //if(currentHeading - TargetHeading >= 6) {
                    nextState = MotorState.BACK_UP_SENSE_BUTTON;
                    StopMove();
                } if(touchSensor3.isPressed()) {
                    StartRightTurn();
                    nextState = MotorState.TURN_RIGHT;
                }
                break;
            case TURN_LEFT:
                StartLefttTurnGentle();
                if(touchSensor3.isPressed() && touchSensor2.isPressed()) {
                    //if(currentHeading - TargetHeading >= 6) {
                    nextState = MotorState.BACK_UP_SENSE_BUTTON;
                    StopMove();
                } if(!touchSensor2.isPressed()) {
                    StartRightTurn();
                    nextState = MotorState.TURN_RIGHT;
                    parkSpeedDelta =- (float) 0.01;
                    leftTurnCount++;
                } else if(waitTimer.time() > 250) {
                    StartRightTurnBackwards();
                    nextState=MotorState.TURN_RIGHT_BACK;
                    leftTurnCount++;
                }
                break;
            case WALL_FOLLOW_BACK_TOUCH:
                StartWallFollow();
                if(touchSensor3.isPressed()) {
                    nextState = MotorState.BACK_UP_SENSE_BUTTON;
                }
                break;
            case WALL_FOLLOW:
                StartWallFollow();
                //DbgLog.msg("InWallFollowState");
                if (lightSensor0.getRawLightDetected() >= 2.0) {
                    nextState = MotorState.STOP_ROBOT ;
                    if (buttonPressCount >= 1) {
                        nextStateAfterWait = MotorState.SENSE_BUTTON ;
                    } else {
                        nextStateAfterWait = MotorState.ARE_WE_DONE ;
                    }
                }
                RobotLog.i("RawLight "+lightSensor0.getRawLightDetected());
                break;
            case WALL_FOLLOW_BACK:
                StartWallFollowBackwards();
                //DbgLog.msg("InWallFollowState");
                if (lightSensor0.getRawLightDetected() >= 2.0) {
                    nextState = MotorState.STOP_ROBOT ;
                    if (buttonPressCount == 2)
                        nextStateAfterWait = MotorState.SENSE_BUTTON ;
                }
                RobotLog.i("RawLight "+lightSensor0.getRawLightDetected());
                break;
            case WALL_FOLLOW_PASSED_TAPE:
                StartWallFollow();
                if (lightSensor0.getRawLightDetected() < 2.0) {
                    nextState = MotorState.WALL_FOLLOW ;
                }
                RobotLog.i("RawLight "+lightSensor0.getRawLightDetected());
                break;
            case WALL_FOLLOW_PAST_TAPE_BACK:
                StartWallFollowBackwards();
                if (lightSensor0.getRawLightDetected() < 1.8) {
                    nextState = MotorState.WALL_FOLLOW_BACK ;
                }
                RobotLog.i("RawLight "+lightSensor0.getRawLightDetected());
                break;

            case SENSE_BUTTON:
                if (SenseBeaconColor(SensorColor.Blue)) {
                    cdim.setLED(0x0,true);
                    waitTimer.reset();
                    SideCurrentPosition = motorSideButton.getCurrentPosition();
                    nextState = MotorState.PUSH_FRONT_BUTTON;
                } else if (SenseBeaconColor(SensorColor.Red)) {
                    SideCurrentPosition = motorSideButton.getCurrentPosition();
                    cdim.setLED(0x1,true);
                    waitTimer.reset();
                    nextState = MotorState.PUSH_BACK_BUTTON;
                } else {
                    RobotLog.i("ERROR: Light not Sensed backup");
                    BackUp();
                    waitTimer.reset();
                    nextState= MotorState.BACK_UP_SENSE_BUTTON;
                }
                break;
            case BACK_UP_SENSE_BUTTON:
                if(SenseBeaconColor(SensorColor.Blue)||SenseBeaconColor(SensorColor.Red)) {
                    StopMove();
                    nextState = MotorState.SENSE_BUTTON;
                    RobotLog.i("Sensed Button");
                } else if(lightSensor0.getRawLightDetected() >= 2.0 ) {
                    nextState = MotorState.STOP_ROBOT ;
                    nextStateAfterWait = MotorState.SENSE_BUTTON ;
                } else if(waitTimer.time()>500){
                    StopMove();
                    nextState= MotorState.WALL_FOLLOW;
                    RobotLog.i("Ran Out of time");
                }
                break;
            case BACK_UP_TO_VORTEX:
                resetValueLeft = -motorLeft1.getCurrentPosition();
                resetValueRight = motorRight1.getCurrentPosition();
                TargetHeading = currentHeading + 15;
                MoveBackward(TargetHeading);
                nextState = MotorState.WAIT_DRIVE_VORTEX;
                break;
            case WAIT_DRIVE_VORTEX:
                driveCorrection = StabilizeMeOnCurrentHeading(currentHeading,TargetHeading);
                motorLeftPower = -normalSpeed - (float) driveCorrection;
                motorRightPower = -normalSpeed + (float) driveCorrection;
                if (pitch <= -3 ) {
                    StopMove();
                    nextState = MotorState.DONE;
                }

                break;
            //TODO Check return to Votex
            case PUSH_FRONT_BUTTON:
                cdim.setDigitalChannelState(LED_CHANNEL,true);
                motorSideButtonPower = (float) 0.5;
                if((motorSideButton.getCurrentPosition() - SideCurrentPosition > 200) || (waitTimer.time() >= 2000)) {
                    motorSideButtonPower = (float) 0.0;
                    nextState = MotorState.ARE_WE_DONE;
                }
                break;
            case PUSH_BACK_BUTTON:
                motorSideButtonPower = (float) -0.5;
                if((motorSideButton.getCurrentPosition() - SideCurrentPosition < -200) || (waitTimer.time() >= 2000)) {
                    motorSideButtonPower = (float) 0.0;
                    nextState = MotorState.ARE_WE_DONE;
                }
                nextState = MotorState.WAIT ;
                nextStateAfterWait = MotorState.ARE_WE_DONE ;
                break;
            case ARE_WE_DONE:
                ResetServoPushers();
                if (buttonPressCount == 0) {
                    buttonPressCount += 1 ;
                    nextState = MotorState.WAIT ;
                    nextStateAfterWait = MotorState.WALL_FOLLOW_PASSED_TAPE ;
                } else if (buttonPressCount == 1){
                    nextState = MotorState.WALL_FOLLOW_PAST_TAPE_BACK ;
                    StartWallFollowBackwards();
                    buttonPressCount += 1 ;
                    nextStateAfterWait = MotorState.DONE;
                } else if (buttonPressCount == 2) {
                    nextState = MotorState.BACK_UP_TO_VORTEX ;
                }
                break;
            case NEXT_BUTTON:
                // not used
                motorLeftPower = normalSpeed ;
                motorRightPower = normalSpeed - (float)0.05 ;
                nextStateAfterWait = MotorState.WALL_FOLLOW;
                break;
            case WAIT:
                if (waitTimer.time() >= 1000) {
                    nextState = nextStateAfterWait ;
                }
                break;
            case STOP_ROBOT:
                StopMove();
                nextState = nextStateAfterWait;
                break;
            case DONE:
                StopMove();
                break;
            case ERROR_STATE:
                StopMove();
                break;
            default:
                break;
        }




        //clips motor and servo power/position
        motorLeftPower = Range.clip(motorLeftPower, motorPowerMin, motorPowerMax);
        motorRightPower = Range.clip(motorRightPower, motorPowerMin, motorPowerMax);
        motorSideButtonPower = Range.clip(motorSideButtonPower, motorPowerMin, motorPowerMax);

        //sets motor and servo power/position
        //DbgLog.msg("LeftPower "+motorLeftPower+"RightPower "+motorRightPower);
        motorLeft1.setPower(motorLeftPower);
        motorLeft2.setPower(motorLeftPower);
        motorRight1.setPower(motorRightPower);
        motorRight2.setPower(motorRightPower);
        motorSideButton.setPower(motorSideButtonPower);




    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });
        telemetry.addLine()
                .addData("Touch3", new Func<String>() {
                    @Override public String value() {
                        if (touchSensor3.isPressed())
                            return "Is Pressed";
                        else {
                            return "Not Pressed";
                        }
                    }})
                    .addData("Touch2", new Func<String>() {
                                @Override public String value() {
                                    if ( touchSensor2.isPressed())
                                        return "Is Pressed";
                                    else {
                                        return "Not Pressed";
                                    }
                                }

                            });
        telemetry.addLine()
                .addData("DriveCorrector", new Func<String>() {
                    @Override
                    public String value() {
                        return formatDouble(driveCorrection);
                    }
                });
        telemetry.addLine()
                .addData("targetHeading ", new Func<String>() {
                    @Override public String value() {
                        return formatDouble(TargetHeading);
                    }
                })
                .addData("currentHeading" , new Func<String>() {
                    @Override public String value() {
                        return formatDouble(currentHeading);
                    }
                })
                .addData("diffFromStartHeading ", new Func<String>() {
                    @Override
                    public String value() {
                        return formatDouble(diffFromStartHeading);
                    }
                });
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, (angles.firstAngle));
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
        telemetry.addLine()
                .addData("Light1 Raw", new Func<String>() {
                    @Override public String value() {
                        return formatDouble(lightSensor0.getRawLightDetected());
                    }
                })
                .addData("Normal", new Func<String>() {
                    @Override public String value() {
                        return formatDouble(lightSensor0.getLightDetected());
                    }
                });
        telemetry.addLine()
                .addData("Motor Power Left1", new Func<String>() {
                    @Override public String value() {
                        return formatDouble(motorLeftPower);
                    }
                })
                .addData("Right1", new Func<String>() {
                    @Override
                    public String value() {
                        return formatDouble(motorRightPower);
                    }
                });
        telemetry.addLine()
                .addData("Position Left", new Func<String>() {
                    @Override
                    public String value() {
                        return formatDouble(positionLeft);
                    }
                })
                .addData("Right", new Func<String>() {
                    @Override public String value() {
                        return formatDouble(positionRight);
                    }
                });
        telemetry.addLine()
                .addData("Side Button", new Func<String>() {
                    @Override
                    public String value() {
                        return formatDouble(sideButtonPushPosition);
                    }
                })
                .addData("Wait Time ", new Func<String>() {
                    @Override
                    public String value() {
                        return formatDouble(waitTimer.time());
                    }});
        telemetry.addLine()
                .addData("Control Count", new Func<String>() {
                    @Override public String value() {
                        return formatDouble(buttonPressCount);
                    }
                });
        telemetry.addLine()
                .addData("currentState", new Func<String>() {
                    @Override
                    public String value() {
                        return currentState.name();
                    }
                });
        telemetry.addLine()
                .addData("ColorBlue", new Func<String>() {
                    @Override
                    public String value() {return formatInt(sensorRGB.blue());
                    }
                })
                .addData("ColorRed", new Func<String>() {
                    @Override
                    public String value() {return formatInt(sensorRGB.red());
                    }
                });
        telemetry.addLine()
                .addData("currentState", new Func<String>() {
                    @Override
                    public String value() {
                        return currentState.name();
                    }
                })
                .addData("nextState", new Func<String>() {
                    @Override
                    public String value() {
                        return nextState.name();
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------
    String formatInt(int inputValue) {
        return String.format("%d", inputValue);
    }

    String formatDouble(double inputValue) {
        return String.format("%.4f", inputValue);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void StartWallFollow() {
        //DbgLog.msg("Start Wall Follow");
        motorLeftPower = normalSpeed ;
        motorRightPower = normalSpeed - (float)0.03;
    }
    public void StartWallFollowBackwards() {
        motorLeftPower = -(normalSpeed );
        motorRightPower = -(normalSpeed - (float)0.03);
    }

    public void StartPivotLeft ()   {
        motorLeftPower = -normalTurnSpeed;
        motorRightPower = normalSpeed;
    }
    public void StartLeftTurn(){
        motorLeftPower = normalTurnSpeed/2;
        motorRightPower = normalTurnSpeed;
    }
    public void StartLefttTurnGentle(){
        motorLeftPower = normalTurnSpeed - parkSpeedDelta;
        motorRightPower = normalTurnSpeed + parkSpeedDelta;
    }
    public void StartRightTurn() {
        motorLeftPower = (float) 0.12;
        motorRightPower = (float) 0.07;
    }
    public void StartRightTurnBackwards(){
        motorLeftPower = (float) -0.12;
        motorRightPower = (float) -0.07;
    }

    public void BackUp(){
        motorLeftPower = -normalTurnSpeed;
        motorRightPower = -normalTurnSpeed;
    }

    public void TurnLeftBack(){
        motorLeftPower = -normalTurnSpeed ;
        motorRightPower = -normalSpeed ;
    }

    // Returning the current Heading before we start moving... We want to continue on this path
    public double MoveForward(double TargetHeading){
        motorLeftPower = normalSpeed;
        motorRightPower = normalSpeed;

        //startOrientation = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
        //startOrientation = startOrientation > (float) 180.0 ? (startOrientation- (float)360.0) : startOrientation;

        motorPID = new PIDController(TargetHeading);
        //DbgLog.msg("Set Target" + startOrientation);
        //DbgLog.msg("Heading" + imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle);
        return TargetHeading;
    }

    public double MoveBackward(double TargetHeading){
        motorLeftPower = -normalSpeed;
        motorRightPower = -normalSpeed;

        //startOrientation = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
        //startOrientation = startOrientation > (float) 180.0 ? (startOrientation- (float)360.0) : startOrientation;

        motorPID = new PIDController(TargetHeading);
        //DbgLog.msg("Set Target" + startOrientation);
        //DbgLog.msg("Heading" + imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle);
        return TargetHeading;
    }

    public void StopMove(){
        motorLeftPower = (float) 0.0;
        motorRightPower = (float) 0.0;
    }

    public void ResetServoPushers(){
        motorSideButtonPower = 0 ;
    }

    public boolean SenseBeaconColor(SensorColor BeaconColor) {
        if (BeaconColor == SensorColor.Blue) {
            if (sensorRGB.blue() > sensorRGB.red()) {
                return true;
            }
        }
        if (BeaconColor == SensorColor.Red) {
            if (sensorRGB.red() > sensorRGB.blue()) {
                return true ;
            }
        }
        return false ;
    }
    public float MotorPosition(DcMotor motor, float resetValue) {
        return(motor.getCurrentPosition() - resetValue) ;
    }

    public boolean AreWeThereYet(int resetValueLeft, int resetValueRight){
        positionLeft = -motorLeft1.getCurrentPosition() - resetValueLeft;
        positionRight = motorRight1.getCurrentPosition() - resetValueRight;
        positionLeft = positionLeft / WheelPositionDivisior;//(wheelDiameter*3.14159265358)
        positionRight = positionRight / WheelPositionDivisior; //(wheelDiameter*3.14159265358)

        return (Math.abs(positionLeft) > normalLine) && (positionRight > normalLine);
    }

    public boolean AreWeTurnedYet(double resetValueHeading) {
        degreesTurned = angles.firstAngle - resetValueHeading;

        return Math.abs(degreesTurned) > normal90turn;
    }

    // Save some stack! This is a functio that is called alot...
    private double xDiff;
    private double yDiff;
    public boolean AreWeStableYet(){
        gravity = imu.getGravity();

        if(prevGravity == null){ //First time we call this function or we are starting a test
            prevGravity = gravity;
            return false;
        }

        xDiff = Math.abs(gravity.xAccel - prevGravity.xAccel);
        yDiff = Math.abs(gravity.yAccel - prevGravity.yAccel);

        if((xDiff < ACCLERATION_STABILITY) && (yDiff < ACCLERATION_STABILITY)){
            prevGravity = null;
            return true;
        }

        prevGravity = gravity;
        return false;
    }

    //Change this if value between readings bounces to much
    private static final float SIGNIFICANT_HEADING_DIFF = 15;
    double diffFromStartHeading;
    private final float CORRECTOR = (float)0.1;

    private double StabilizeMeOnCurrentHeading(double currentHeading, double target_){
        double correction = 0.0;
        //currentHeading = currentHeading > (float) 180.0 ? currentHeading -(float) 360.0 : currentHeading;
        diffFromStartHeading = target_ - currentHeading;
        //DbgLog.msg("TargetHeading"+target_+" currentHD "+ currentHeading);
        if(false) {
            correction = (diffFromStartHeading > SIGNIFICANT_HEADING_DIFF) ? (diffFromStartHeading / 180) * CORRECTOR : (diffFromStartHeading / 180) * CORRECTOR;
        } else { //PID controller
            correction = motorPID.Update(currentHeading);
            //headingCorrectorLeft = (correction < -SIGNIFICANT_HEADING_DIFF) ? -1 * correction : -1 * correction;
            //headingCorrectorRight = (correction > SIGNIFICANT_HEADING_DIFF) ? correction : correction;
        }
        return(correction);
    }

    @Override
    public void stop() {
        motorLeft1.setPower(0);
        motorLeft2.setPower(0);
        motorRight1.setPower(0);
        motorRight2.setPower(0);
        cdim.setLED(0x0,false);
        cdim.setLED(0x1,false);
        cdim.setDigitalChannelState(LED_CHANNEL,false);
    }

    class PIDController{
        private PIDController() {}
        public PIDController(double setPoint){
            setPoint_ = setPoint;
            lastError_ = 0;
            lastTime_ = System.currentTimeMillis();
            errorSum_ = 0;

            //kp_ = (float)0.0025 ;
            kp_ = (float)0.0015 ;
            ki_ = (float)0;
            kd_ = (float)0;
        }

        public PIDController(double setPoint, double kp, double ki, double kd){
            setPoint_ = setPoint;
            lastError_ = 0;
            lastTime_ = System.currentTimeMillis();
            errorSum_ = 0;

            kp_ = kp;
            ki_ = ki;
            kd_ = kd;
        }

        private double lastError_;
        private double setPoint_;
        private double errorSum_;
        private double kp_;
        private double ki_;
        private double kd_;
        private long lastTime_;

        public double Update(double newInput){
            long time = System.currentTimeMillis();
            long period = time - lastTime_;
            double error  = setPoint_ - newInput;
            errorSum_ += (error * period);
            double derError = 0;//(error - lastError_) / period;

            double output = (kp_ * error) + (ki_ * errorSum_) + (kd_ * derError);

            lastError_ = error;
            lastTime_ = time;
            //DbgLog.msg("Drive Correction " + output);
            return output;
        }
    }
}


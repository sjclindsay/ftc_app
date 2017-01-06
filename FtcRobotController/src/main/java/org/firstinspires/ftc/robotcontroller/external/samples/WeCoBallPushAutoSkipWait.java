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

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

@Autonomous(name="WeCo: WeCoBallPushAutoSkipWait", group="WeCo")
// @Disabled
public class WeCoBallPushAutoSkipWait extends OpMode  {
    public enum MotorState{
        ERROR_STATE,
        WAIT_TO_GO,
        WAIT_TO_START,
        WAIT_PERIOD,
        WAIT_START_PERIOD,
        STOP_MOVING,
        WAIT_FOR_STABLE,
        DRIVE_FORWARD_TO_BALL,
        WAIT_DRIVE_FORWARD,
        SENSE_BALL,
        PUSH_OFF_BALL,
        ARE_WE_DONE,
        DONE
    }
    static double startWaitPeriod = 0.0;
    //Drive Control Values
    static final float normalTurnSpeed = (float) 0.05;
    static final float normalSpeed = (float) 0.15;
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

    private static double SERVOLEFTRIGHT_STARTPOSITION = 0.5;
    private static double SERVOUPDOWN_STARTPOSITION = 0.2;
    private static double SERVOPUSHBUTTON_STARTPOSITION = 0.1;

    TouchSensor touchSensor1;  // Hardware Device Object
    LightSensor lightSensor1;  // Hardware Device Object
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

    int resetValueLeft = 0;
    int resetValueRight = 0;
    double resetValueHeading = 0;
    double degreesTurned = 0;
    float motorPowerMin = -1;
    float motorPowerMax = 1;
    double positionLeft = 0;
    double positionRight = 0;
    float motorLeftPower = 0;
    float motorRightPower = 0;
    double driveCorrection = 0.0;
    PIDController motorPID;
    Orientation RobotAngles;
    double currentHeading = 0.0;
    double pitch = 0.0 ;
    float startOrientation;
    ElapsedTime StabilizationTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private double TargetHeading;

    ElapsedTime elapsedTime = new ElapsedTime();

    public WeCoBallPushAutoSkipWait() {
    }

    @Override
    public void init() {

        // get a reference to our Hardware objects
        touchSensor1 = hardwareMap.touchSensor.get("touchSensorP1");
        lightSensor1 = hardwareMap.lightSensor.get("lightSensorP0");
        servoPushButton = hardwareMap.servo.get("servoButtonP3");
        servoLeftRight = hardwareMap.servo.get("servoLeftRightP1");
        servoUpDown = hardwareMap.servo.get("servoUpDownP2");
        servoTapeLeft = hardwareMap.servo.get("servoTapeLeft");
        servoTapeRight = hardwareMap.servo.get("servoTapeRight");

        motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight1 = hardwareMap.dcMotor.get("motorRight1");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");

        //Setup Hardware
        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);

        servoPushButton.setPosition(SERVOPUSHBUTTON_STARTPOSITION);
        servoLeftRight.setPosition(SERVOLEFTRIGHT_STARTPOSITION);
        servoUpDown.setPosition(SERVOUPDOWN_STARTPOSITION);
        servoTapeLeft.setPosition(0.5);
        servoTapeRight.setPosition(0.5);
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional inf
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
        composeTelemetry();
    }
    //Event Control Value
    double count;
    double etime;

    @Override
    public void start() {
        lightSensor1.enableLed(true);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 200);
    }

    @Override
    public void loop() {

        etime = elapsedTime.time();

        telemetry.update();
        currentState = nextState;
        pitch = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.thirdAngle)) ;
        //RobotAngles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        currentHeading = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle));
        switch(nextState) {
            case WAIT_START_PERIOD:
                StabilizationTimer.startTime();
                nextState = MotorState.WAIT_PERIOD;
                break;
            case WAIT_PERIOD:
                if(StabilizationTimer.time() > 15000) {
                    nextState = MotorState.WAIT_DRIVE_FORWARD;
                }
                break;
            case WAIT_TO_START:
                nextState = MotorState.DRIVE_FORWARD_TO_BALL;
                break;
            case DRIVE_FORWARD_TO_BALL:
                resetValueLeft = -motorLeft1.getCurrentPosition();
                resetValueRight = motorRight1.getCurrentPosition();
                TargetHeading = currentHeading;
                MoveForward(TargetHeading);
                nextState = MotorState.WAIT_DRIVE_FORWARD;
                break;
            case WAIT_DRIVE_FORWARD:
                driveCorrection = StabilizeMeOnCurrentHeading(currentHeading,TargetHeading);
                motorLeftPower = normalSpeed - (float) driveCorrection;
                motorRightPower = normalSpeed + (float) driveCorrection;
                if(pitch > 2.00) {
                    nextState = MotorState.DONE;
                }
                break;
            case SENSE_BALL:
                //turn untill light sensor is solid
                nextState = MotorState.PUSH_OFF_BALL;
                break;
            case PUSH_OFF_BALL:
                //go until y axis is stabilized
                if (pitch > 2.00) {
                    nextState = MotorState.DONE ;
                }
                break;
            case STOP_MOVING:
                StopMove();
                nextState = MotorState.WAIT_FOR_STABLE;
                break;
            case WAIT_FOR_STABLE:
                if(AreWeStableYet()) {
                    nextState = nextStateAfterWait;
                }
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

        //sets motor and servo power/position

        motorLeft1.setPower(motorLeftPower);
        motorLeft2.setPower(motorLeftPower);
        motorRight1.setPower(motorRightPower);
        motorRight2.setPower(motorRightPower);

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
                        return formatDouble(lightSensor1.getRawLightDetected());
                    }
                })
                .addData("Normal", new Func<String>() {
                    @Override public String value() {
                        return formatDouble(lightSensor1.getLightDetected());
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
                .addData("Control Count", new Func<String>() {
                    @Override public String value() {
                        return formatDouble(count);
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
    String formatDouble(double inputValue) {
        return String.format("%.4f", inputValue);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void StartLeftTurn(){
        motorLeftPower = -normalTurnSpeed;
        motorRightPower = normalTurnSpeed;
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



    public void StopMove(){
        motorLeftPower = (float) 0.0;
        motorRightPower = (float) 0.0;

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
    }

    class PIDController{
        private PIDController() {}
        public PIDController(double setPoint){
            setPoint_ = setPoint;
            lastError_ = 0;
            lastTime_ = System.currentTimeMillis();
            errorSum_ = 0;

            kp_ = (float)0.0025 ;
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
            DbgLog.msg("Drive Correction " + output);
            return output;
        }
    }
}


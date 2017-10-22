package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareColorSensor;
import org.firstinspires.ftc.robotcontroller.external.samples.WeCoBallPushAuto;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.FormatHelper;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.teamcode.FormatHelper.formatDouble;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  11  drive motor:        "drive_wheel_11"
 * Motor channel:  01 drive motor:        "drive_wheel_01"
 * Motor channel:  10 drive motor:        "drive_wheel_10"
 * Motor channel:  00 drive motor:        "drive_wheel_00"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"

 */

enum robotHWconnected {
    MotorOnly,
    MotorGyro,
    MotorGyroServo,
    MotorGyroLifter,
    MotorGyroLifterVufor,
    MotorGyroLifterVuforColor
}

enum PIDAxis {
    gyro,
    tx,
    ty,
    tz,
    rx,
    ry,
    rz
}

public class HardwareOmniBot
{
    /* Public OpMode members. */
    private robotHWconnected connectedHW = robotHWconnected.MotorGyroLifterVufor;
    protected DcMotor Motor00   = null;
    protected DcMotor  Motor01  = null;
    protected DcMotor  Motor10   = null;
    protected DcMotor  Motor11  = null;
    protected Servo servoJewel = null ;
    protected static final float motorPowerMin = -1 ;
    protected static final float motorPowerMax = 1 ;
    protected  float gamePad1LeftStickMagnitude = 0 ;
    protected  double maxPower = 1;
    protected boolean gyroConnected = false;
    protected boolean lifterConnected = false ;
    protected boolean vuForConnected = false ;
    protected boolean colorConnected = false ;
    protected boolean servoConnected = false;
    protected HardwareColorSensor colorSensor = null ;
    protected HardwareGyro gyroScope = null;
    protected HardwareLifter lifter = null ;
    protected HardwareVuforia vufor = null ;
    protected double TargetHeading = 0.0;
    private PIDController motorPID = null;
    private boolean FirstCallPIDDrive = true;
    private double correction = 0.0 ;



    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareOmniBot(){
    }

    public HardwareOmniBot(robotHWconnected ConnectedParts){
        if((ConnectedParts == robotHWconnected.MotorGyro) || (ConnectedParts == robotHWconnected.MotorGyroServo)){
            gyroConnected = true;
        }
        if (ConnectedParts == robotHWconnected.MotorGyroLifter) {
            gyroConnected = true ;
            lifterConnected = true ;
        }
        if (ConnectedParts == robotHWconnected.MotorGyroLifterVufor) {
            gyroConnected = true ;
            lifterConnected = true ;
            vuForConnected = true ;
        }
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        Motor00  = hwMap.dcMotor.get("drive_wheel_00");
        Motor01  = hwMap.dcMotor.get("drive_wheel_01");
        Motor10  = hwMap.dcMotor.get("drive_wheel_10");
        Motor11  = hwMap.dcMotor.get("drive_wheel_11");
        servoJewel = hwMap.servo.get("servoJewel") ;

        Motor10.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        Motor11.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
       // colorSensor = hwMap.colorSensor.get("colorSensor1");
        if(gyroConnected) {
            gyroScope = new HardwareGyro();
            RobotLog.i("defined gyroscope");
            gyroScope.init(hwMap);
        }
        if (lifterConnected) {
            lifter = new HardwareLifter(LifterHWcontroller.LifterGrabber);
            RobotLog.i("defined lifter");
            lifter.init(hwMap);
        }
        if (vuForConnected) {
            vufor = new HardwareVuforia() ;
            RobotLog.i("defined Vufor") ;
        }

        // Set all motors to zero power
        Motor00.setPower(0);
        Motor01.setPower(0);
        Motor10.setPower(0);
        Motor11.setPower(0);
        servoJewel.setPosition(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        Motor00.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor01.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor10.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor11.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void start() {
        if (gyroConnected) {
            gyroScope.start();
        }
        if (lifterConnected) {
            lifter.start();
        }
        if (vuForConnected) {
            vufor.start();
        }
    }

    public void lowerServoJewel () { servoJewel.setPosition(0.5) ; }
    public void raiseServoJewel () { servoJewel.setPosition(0) ; }

    public void setBotMovement (double motorPower00, double motorPower01, double motorPower10, double motorPower11) {


        motorPower00 = Range.clip(motorPower00, motorPowerMin, motorPowerMax);
        motorPower01 = Range.clip(motorPower01, motorPowerMin, motorPowerMax);
        motorPower10 = Range.clip(motorPower10, motorPowerMin, motorPowerMax);
        motorPower11 = Range.clip(motorPower11, motorPowerMin, motorPowerMax);

        Motor00.setPower(motorPower00);
        Motor01.setPower(motorPower01);
        Motor10.setPower(motorPower10);
        Motor11.setPower(motorPower11);
    }

    public double getcurrentHeading() {
        return(gyroScope.currentHeadingZ);
    }

    public void resetFirstPIDDrive () { FirstCallPIDDrive = true; }

    public void gyroDriveStaight(double Speed, double targetHeading) {
        double correction = 0.0;
        double currentDiff = 0.0;
        if(FirstCallPIDDrive) {
            RobotLog.i("Set up PID Target " + targetHeading);
            RobotLog.i("Current Heading" + gyroScope.currentHeadingZ);
            motorPID = new PIDController(targetHeading);
            FirstCallPIDDrive = false;
        }

        currentDiff = targetHeading - gyroScope.currentHeadingZ;
        correction = motorPID.Update(gyroScope.currentHeadingZ);

        double motorPower00 = Range.clip(Speed-correction, motorPowerMin, motorPowerMax);
        double motorPower01 = Range.clip(Speed-correction, motorPowerMin, motorPowerMax);
        double motorPower10 = Range.clip(Speed+correction, motorPowerMin, motorPowerMax);
        double motorPower11 = Range.clip(Speed+correction, motorPowerMin, motorPowerMax);

        Motor00.setPower(motorPower00);
        Motor01.setPower(motorPower01);
        Motor10.setPower(motorPower10);
        Motor11.setPower(motorPower11);


    }

    public void gyroDriveOmniStaight(double power00, double power01, double power10, double power11, double targetHeading) {
        double currentDiff = 0.0;

        if(FirstCallPIDDrive) {
            RobotLog.i("Set up PID Target " + targetHeading);
            RobotLog.i("Current Heading" + gyroScope.currentHeadingZ);

            motorPID = new PIDController(targetHeading, 0.0055, 0, 0);
            FirstCallPIDDrive = false;
        }

        currentDiff = targetHeading - gyroScope.currentHeadingZ;
        correction = motorPID.Update(gyroScope.currentHeadingZ);


        power00 += correction;
        power01 += correction;
        power10 -= correction;
        power11 -= correction;

        setBotMovement(power00, power01, power10, power11) ;

    }

    public void txSquareOmniBot ( float targetHeading ) {
        double currentHeadingTX = 0.0;

        vufor.updateVuforiaCoords();
        currentHeadingTX = vufor.getVuforiaCoords(HardwareVuforia.vuForiaCoord.tX) ;



        if(FirstCallPIDDrive) {
            RobotLog.i("Set up PID Target " + targetHeading);
            RobotLog.i("Current Heading" + currentHeadingTX);

            motorPID = new PIDController(targetHeading, 0.003, 0, 0);
            FirstCallPIDDrive = false;
        }

        correction = motorPID.Update(currentHeadingTX);


        setBotMovement(correction, correction, correction, correction) ;
    }

    public void rxSquareOmniBot ( float targetHeading ) {
        double currentHeadingRX = 0.0;

        vufor.updateVuforiaCoords();
        currentHeadingRX = vufor.getVuforiaCoords(HardwareVuforia.vuForiaCoord.rX) ;



        if(FirstCallPIDDrive) {
            RobotLog.i("Set up PID Target " + targetHeading);
            RobotLog.i("Current Heading" + currentHeadingRX);

            motorPID = new PIDController(targetHeading, 0.003, 0, 0);
            FirstCallPIDDrive = false;
        }

        correction = motorPID.Update(currentHeadingRX);


        setBotMovement(correction, correction, -correction, -correction) ;
    }

    public void tzSquareOmniBot ( float targetHeading ) {
        double currentHeadingTZ = 0.0;

        vufor.updateVuforiaCoords();
        currentHeadingTZ = vufor.getVuforiaCoords(HardwareVuforia.vuForiaCoord.tZ) ;



        if(FirstCallPIDDrive) {
            RobotLog.i("Set up PID Target " + targetHeading);
            RobotLog.i("Current Heading" + currentHeadingTZ);

            motorPID = new PIDController(targetHeading, 0.003, 0, 0);
            FirstCallPIDDrive = false;
        }

        correction = motorPID.Update(currentHeadingTZ);


        setBotMovement(correction, correction, -correction, -correction) ;
    }

    public void driveOmniBot (float magnitude, float direction, float targetHeading, PIDAxis axis) {
        if (axis == PIDAxis.gyro) {

            float yValue = (float) Math.cos( direction*(Math.PI/180) ) ;
            float xValue = (float) Math.sin( direction*(Math.PI/180) ) ;

            RobotLog.i("cos is " + yValue);
            RobotLog.i("sin is " + xValue);

            float power00 = (yValue - xValue)*magnitude ;
            float power01 = (yValue + xValue)*magnitude ;
            float power10 = (yValue + xValue)*magnitude ;
            float power11 = (yValue - xValue)*magnitude ;

            gyroDriveOmniStaight(power00, power01, power10, power11, targetHeading);
        } else if (axis == PIDAxis.tx) {
            txSquareOmniBot(targetHeading);
        } else if (axis == PIDAxis.rx) {
            rxSquareOmniBot(targetHeading);
        } else if (axis == PIDAxis.tz) {
            tzSquareOmniBot(targetHeading);
        }


    }

    public void setLifterGrabber (float lifterSpeed) {
        lifter.setLifterGrabber(lifterSpeed);
    }
    public void setLifterGrabber (float lifterSpeed, double grabberPosition) {

        grabberPosition = HardwareGrabber.servoGrabberInitialPosition - grabberPosition*0.35;

        lifter.setLifterGrabber(lifterSpeed, grabberPosition);
    }

    public double vuforiaCoordinates (HardwareVuforia.vuForiaCoord axis) {return vufor.getVuforiaCoords(axis) ;}

    public void addTelemetry(Telemetry telemetry) {

        if(gyroConnected){
            gyroScope.addTelemetry(telemetry);
            telemetry.addLine()
                    .addData("targetHeading ", new Func<String>() {
                        @Override public String value() {
                            return formatDouble(TargetHeading);
                        }
                    })
                    .addData("currentHeading" , new Func<String>() {
                        @Override public String value() {
                            return formatDouble(gyroScope.currentHeadingZ);
                        }
                    })
                    .addData("Current Error", new Func<String>() {
                                @Override public String value() {
                                    return formatDouble(correction) ;
                                }}) ;
        }

        telemetry.addLine()
            .addData("Motor Power 00", new Func<String>() {
                @Override
                public String value() {
                    return FormatHelper.formatDouble(Motor00.getPower());
                }
            })
            .addData("Motor Power 01", new Func<String>() {
                @Override
                public String value() {
                    return FormatHelper.formatDouble(Motor01.getPower());
                }
            });
        telemetry.addLine()
            .addData("Motor Power 10", new Func<String>() {
                @Override
                public String value() {
                    return FormatHelper.formatDouble(Motor10.getPower());
                }
            })
            .addData("Motor Power 11", new Func<String>() {
                @Override
                public String value() {
                    return FormatHelper.formatDouble(Motor11.getPower());
                }
            }) ;
        if(lifterConnected) {
            lifter.addTelemetry(telemetry);
        }
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */

    public void waitForTick(long periodMs) {
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
        if(gyroConnected) {
           gyroScope.Update();
        }


    }
    public double maxPowerIdentifier (double motorPower00, double motorPower01, double motorPower10, double motorPower11) {
        double power1 = Math.max(motorPower00,motorPower01) ;
        double power2 = Math.max(motorPower10,motorPower11) ;
        double maxPower = Math.max(power1,power2) ;
        return maxPower ;
    }
}


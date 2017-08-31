package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
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

 */

enum robotHWconnected {
    MotorOnly,
    MotorGyro,
    MotorGyroServo,
    MotorServo
}

public class HardwareOmniBot
{
    /* Public OpMode members. */
    private robotHWconnected connectedHW = robotHWconnected.MotorOnly;
    protected DcMotor Motor00   = null;
    protected DcMotor  Motor01  = null;
    protected DcMotor  Motor10   = null;
    protected DcMotor  Motor11  = null;
    protected float motorPowerMin = -1 ;
    protected float motorPowerMax = 1 ;
    protected  float gamePad1LeftStickMagnitude = 0 ;
    protected  double maxPower = 1;
    protected boolean gyroConnected = false;
    protected boolean servoConnected = false;
    protected ColorSensor colorSensor = null;
    protected HardwareGyro gyroScope = null;
    public float currentHeading = (float) 0.0;
    public Acceleration gravity = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareOmniBot(){
    }

    public HardwareOmniBot(robotHWconnected gConnected){
        if((gConnected == robotHWconnected.MotorGyro) || (gyroConnected == robotHWconnected.MotorGyroServo){
            gyroConnected = true;
        }
        if((gConnected == robotHWconnected.MotorGyroServo)||(gConnected == robotHWconnected.MotorServo)) {
            servoConnected = true;
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

        Motor10.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        Motor11.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
       // colorSensor = hwMap.colorSensor.get("colorSensor1");
        if(gyroConnected) {
            gyroScope = new HardwareGyro();
            gyroScope.init(hwMap);
        }

        // Set all motors to zero power
        Motor00.setPower(0);
        Motor01.setPower(0);
        Motor10.setPower(0);
        Motor11.setPower(0);

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
    }


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

    public void complexOmniBotMath (float padLeftStickY, float padLeftStickX, float padRightStickx) {
        double motorPower00  ;
        double motorPower01 ;
        double motorPower10 ;
        double motorPower11 ;

        gamePad1LeftStickMagnitude = (float) Math.pow((padLeftStickX*padLeftStickX +padLeftStickY*padLeftStickY), 0.5) ;
        if (padLeftStickX == -padLeftStickY) {
            motorPower00= 0 ;
            motorPower11 = 0 ;
        } else {
            motorPower00 = gamePad1LeftStickMagnitude*((padLeftStickY+padLeftStickX)/(Math.abs(padLeftStickY+padLeftStickX)));
            motorPower11 = gamePad1LeftStickMagnitude*((padLeftStickY+padLeftStickX)/(Math.abs(padLeftStickY+padLeftStickX)));
        }
        if (padLeftStickX == padLeftStickY) {
            motorPower01 = 0 ;
            motorPower10 = 0 ;
        } else {
            motorPower01 = gamePad1LeftStickMagnitude*((padLeftStickY-padLeftStickX)/(Math.abs(padLeftStickY-padLeftStickX)));
            motorPower10 = gamePad1LeftStickMagnitude*((padLeftStickY-padLeftStickX)/(Math.abs(padLeftStickY-padLeftStickX)));
        }

        motorPower00 = motorPower00 - padRightStickx ;
        motorPower01 = motorPower01 - padRightStickx ;
        motorPower10 = motorPower10 + padRightStickx ;
        motorPower11 = motorPower11 + padRightStickx ;

        if (motorPower00 > 1 || motorPower01 > 1 || motorPower10 > 1 || motorPower11 > 1) {
            maxPower = maxPowerIdentifier(motorPower00, motorPower01, motorPower10, motorPower11) ;
            motorPower00 = motorPower00/maxPower ;
            motorPower01 = motorPower10/maxPower ;
            motorPower10 = motorPower10/maxPower ;
            motorPower11 = motorPower11/maxPower ;
        }

        motorPower00 = Range.clip(motorPower00, motorPowerMin, motorPowerMax);
        motorPower01 = Range.clip(motorPower01, motorPowerMin, motorPowerMax);
        motorPower10 = Range.clip(motorPower10, motorPowerMin, motorPowerMax);
        motorPower11 = Range.clip(motorPower11, motorPowerMin, motorPowerMax);

        Motor00.setPower(motorPower00);
        Motor01.setPower(motorPower01);
        Motor10.setPower(motorPower10);
        Motor11.setPower(motorPower11);
    }

    public void gyroDrive(double Speed, double targetHeading) {


    }

    public void getTelemetry(Telemetry telemetry) {

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
            });
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

    }
    public double maxPowerIdentifier (double motorPower00, double motorPower01, double motorPower10, double motorPower11) {
        double power1 = Math.max(motorPower00,motorPower01) ;
        double power2 = Math.max(motorPower10,motorPower11) ;
        double maxPower = Math.max(power1,power2) ;
        return maxPower ;
    }
}


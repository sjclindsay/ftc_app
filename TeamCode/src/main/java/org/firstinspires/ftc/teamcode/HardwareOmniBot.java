package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

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

public class HardwareOmniBot
{
    /* Public OpMode members. */
    protected DcMotor Motor00   = null;
    protected DcMotor  Motor01  = null;
    protected DcMotor  Motor10   = null;
    protected DcMotor  Motor11  = null;
    protected float motorPowerMin = -1 ;
    protected float motorPowerMax = 1 ;
    protected  float gamePad1LeftStickMagnitude = 0 ;
    protected  double maxPower = 1;
    protected ColorSensor colorSensor = null;
    protected BNO055IMU imu = null;
    protected BNO055IMU.Parameters parameters = null;
    public float currentHeading = (float) 0.0;
    public Acceleration gravity = null;


    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareOmniBot(){

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

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        /*parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        */
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        //imu = hwMap.get(BNO055IMU.class, "imu");
        //imu.initialize(parameters);


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
        imu.startAccelerationIntegration(new Position(), new Velocity(), 200);
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
    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */

    public void waitForTick(long periodMs) {
        Orientation angles = null;
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
        //angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        //currentHeading = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle));
        //gravity = imu.getGravity();
    }
    public double maxPowerIdentifier (double motorPower00, double motorPower01, double motorPower10, double motorPower11) {
        double power1 = Math.max(motorPower00,motorPower01) ;
        double power2 = Math.max(motorPower10,motorPower11) ;
        double maxPower = Math.max(power1,power2) ;
        return maxPower ;
    }
}


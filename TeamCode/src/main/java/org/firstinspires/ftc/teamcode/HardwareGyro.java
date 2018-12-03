package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FormatHelper;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a GyroScope.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *

 */


public class HardwareGyro {
    /* Public OpMode members. */
    BNO055IMU imu ;
    protected BNO055IMU.Parameters parameters = null;
    public float currentHeadingZ = (float) 0.0;
    public float currentHeadingY = (float) 0.0;
    public float currentHeadingX = (float) 0.0;
    public double currentAccelerationZ = 0.0;
    public double currentAccelerationY = 0.0;
    public double currentAccelerationX = 0.0;
    public boolean gyroInitialized = false ;
    private Orientation angles = null;
    private Acceleration acceleration = null ;
    private Acceleration gravity = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public HardwareGyro() {

    }


    private Thread gyroInitThread = null ;

    public boolean gyroInitThreadIsAlive() {
        return gyroInitThread.isAlive() ;
    }

    private Runnable gyroInitThread1 = new Runnable() {
        @Override
        public void run() {
            RobotLog.i("gyro initializing");

            parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu"
            imu = hwMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            RobotLog.i("gyro initialized");
            gyroInitialized = true ;
        }
    } ;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        gyroInitThread = new Thread(gyroInitThread1) ;
        gyroInitThread.start();

    }

    public void start() {


        //imu.startAccelerationIntegration(new Position(), new Velocity(), 200);
    }

    public void addTelemetry(Telemetry telemetry){
        /*telemetry.addLine()
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
                */
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatedAngleZ();
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatedAngleY();
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatedAngleX();
                    }
                });
        /*telemetry.addLine()
                .addData("AccelerationX", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatDouble(currentAccelerationX) ;
                    }
                })
                .addData("AccelerationY", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatDouble(currentAccelerationY) ;
                    }
                }).addData("AccelerationZ", new Func<String>() {
            @Override
            public String value() {
                return FormatHelper.formatDouble(currentAccelerationZ) ;
            }
        }) */
        ;

    }

    public void Update (){

        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        currentHeadingZ = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        currentHeadingY = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.secondAngle));
        currentHeadingX = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.thirdAngle));
        gravity = imu.getGravity();

        RobotLog.i("Gyro Heading: Z " + formatedAngleZ() + ", Y " + formatedAngleY() + ", X " + formatedAngleX() ) ;
    }
    public String formatedAngleZ() {
        return FormatHelper.formatAngle(AngleUnit.DEGREES,currentHeadingZ);
    }
    public String formatedAngleY() {
        return FormatHelper.formatAngle(AngleUnit.DEGREES,currentHeadingY);
    }
    public String formatedAngleX() {
        return FormatHelper.formatAngle(AngleUnit.DEGREES,currentHeadingX);
    }

    public void UpdateAcceleration () {
        acceleration = imu.getLinearAcceleration().toUnit(DistanceUnit.METER);
        currentAccelerationZ = acceleration.zAccel ;
        currentAccelerationY = acceleration.yAccel ;
        currentAccelerationX = acceleration.xAccel ;
        RobotLog.i("Gyro Acceleration: Z " + currentAccelerationZ + ", Y " + currentAccelerationY + ", X " + currentAccelerationX) ;

    }


}
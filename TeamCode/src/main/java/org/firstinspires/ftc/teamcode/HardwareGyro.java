package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    private Orientation angles = null;
    private Acceleration gravity = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public HardwareGyro() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        DbgLog.msg("gyro initialized");
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu"
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public void start() {

        imu.startAccelerationIntegration(new Position(), new Velocity(), 200);
    }

    public void addTelemetry(Telemetry telemetry){
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
    }

    public void Update (){
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        currentHeadingZ = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        currentHeadingY = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.secondAngle));
        currentHeadingX = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.thirdAngle));
        gravity = imu.getGravity();
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

}
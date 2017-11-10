package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by conno on 11/9/2017.
 */

public class gyroInit implements Runnable {

    private BNO055IMU imu = null ;
    private BNO055IMU.Parameters parameters = null ;
    private HardwareMap hwMap = null ;

    public gyroInit (BNO055IMU _imu, BNO055IMU.Parameters _parameters, HardwareMap _hwMap) {
        imu = _imu ;
        parameters = _parameters ;
        hwMap = _hwMap ;
    }

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
    }

}

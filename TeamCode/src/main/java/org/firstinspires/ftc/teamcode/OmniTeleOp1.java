package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.FormatHelper;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

/**
 * Created by conno on 8/17/2017.
 */

@TeleOp(name="Omni: TeleOp1", group="Omni")


public class OmniTeleOp1 extends OpMode {

    float motorLeft1power = 0;
    float motorLeft2power = 0;
    float motorRight1power = 0;
    float motorRight2power = 0;
    float motorLifterPower = 0 ;
    float servoGrabberPosition = 0 ;
    float leftStickY = 0 ;
    boolean controller1 = true;
    boolean controller2 = false ;
    float dPadScalar = 1 ;
    HardwareOmniBot OmniBot ;
    boolean waitForUpRelease = false ;
    boolean waitForDownRelease = false ;
    protected  float gamePad1LeftStickMagnitude = 0 ;
    protected  double maxPower = 1;
    float [] polarCoordinates = {0,0} ;
    float [] currentPolarCoordinates = {0,0} ;
    float stickAngle = 0 ;
    double targetHeading = 0.0 ;
    boolean triggerLeftToggle = false ;
    int dPadLeftOff = 0 ;
    boolean triggerRightToggle = false ;
    int  dPadRightOff = 0 ;
    boolean headingToggle = false ;
    boolean directionToggle  = false ;

    @Override
    public void init() {
        OmniBot = new HardwareOmniBot(robotHWconnected.MotorGyroLifterCrypto) ;
        OmniBot.init(hardwareMap);
        waitForUpRelease = false ;
        waitForUpRelease = false ;
        dPadScalar = 1 ;
        motorLeft1power = 0;
        motorLeft2power = 0;
        motorRight1power = 0;
        motorRight2power = 0;
        motorLifterPower = 0 ;
        composeTelemetry();
    }

    @Override
    public void start() {

        OmniBot.start();
    }

    @Override
    public void loop() {
        leftStickY = -gamepad1.left_stick_y ;

        dPadScalar = dPadScale(gamepad1.dpad_up,gamepad1.dpad_down,dPadScalar) ;

        //currentPolarCoordinates = getCurrentPolarCoordinate(-gamepad2.left_stick_y, gamepad2.left_stick_x) ;
        //currentPolarCoordinates[0] = currentPolarCoordinates[0]/dPadScalar ;

        //targetHeading -= gamepad2.right_stick_x ;

        if (gamepad1.left_trigger >= 0.5 && !triggerLeftToggle) {
            triggerLeftToggle = true ;
            dPadLeftOff = (dPadLeftOff + 1)%2 ;
        } else if (gamepad1.left_trigger <= 0.5 && triggerLeftToggle) {
            triggerLeftToggle = false ;
        }
        if (gamepad1.right_trigger >= 0.5 && !triggerRightToggle) {
            triggerRightToggle = true ;
            dPadRightOff = (dPadRightOff + 1)%2 ;
        } else if (gamepad1.right_trigger <= 0.5 && triggerRightToggle) {
            triggerRightToggle = false ;
        }


            motorLeft1power = (leftStickY  + gamepad1.left_stick_x - gamepad1.right_stick_x)/dPadScalar ;
            motorLeft2power = (leftStickY  - gamepad1.left_stick_x - gamepad1.right_stick_x)/dPadScalar;
            motorRight1power = (leftStickY - gamepad1.left_stick_x + gamepad1.right_stick_x)/dPadScalar;
            motorRight2power = (leftStickY + gamepad1.left_stick_x + gamepad1.right_stick_x)/dPadScalar;

/*            if (dPadLeftOff == 1) {
                motorLeft1power = 0 ;
                motorLeft2power = 0 ;
            }
            if (dPadRightOff == 1) {
                motorRight1power = 0 ;
                motorRight2power = 0 ;
            } */

            if ((gamepad2.left_bumper) && (!gamepad2.right_bumper)) {
                motorLifterPower = (float) 1.0;
            } else if ((gamepad2.right_bumper) && (!gamepad2.left_bumper)){
                motorLifterPower = (float)-1.0 ;
            } else {
                motorLifterPower = (float) 0.0;
            }

        if (OmniBot.lifter.lifterRangeUpper.getState()) {
            //motorLifterPower = Range.clip(motorLifterPower, -1, 0) ;
        }
        if (OmniBot.lifter.lifterRangeLower.getState()) {
            //motorLifterPower = Range.clip(motorLifterPower,0,1) ;
        }

            OmniBot.setBotMovement(motorLeft1power, motorLeft2power, motorRight1power, motorRight2power);
            OmniBot.setLifterGrabber(motorLifterPower, gamepad2.left_trigger, gamepad2.right_trigger);


        OmniBot.waitForTick(40);
        telemetry.update();

    }

    @Override
    public void stop() {

    }
    void composeTelemetry() {
        OmniBot.addTelemetry(telemetry);
    }


    public float dPadScale (boolean dPadUpValue, boolean dPadDownValue, float dPadScalar) {
        if (dPadUpValue && !waitForUpRelease) {
            waitForDownRelease = true ;
            dPadScalar -= 2 ;
        } else if (!dPadUpValue && waitForUpRelease) {
            waitForUpRelease = false ;
        }
        if (dPadDownValue && !waitForDownRelease) {
            waitForDownRelease = true ;
            dPadScalar += 2 ;
        } else if (!dPadDownValue && waitForDownRelease) {
            waitForDownRelease = false ;
        }
        dPadScalar = Range.clip(dPadScalar,1, 21) ;
        return dPadScalar ;
    }

    public double normalizeAngle(double angle) {
        if (angle > 180) {
            angle -= 360 ;
        }
        if (angle < -180) {
            angle += 360 ;
        }
        return angle ;
    }

    public float [] getCurrentPolarCoordinate (float padStickLeftY, float padStickLeftX) {
        if (padStickLeftX == 0 ) {
            if (padStickLeftY >= 0) {
                stickAngle = 90;
            } else  {
                stickAngle = -90 ;
            }
        } else {
            stickAngle = (float) ( Math.atan( padStickLeftY/padStickLeftX)*(180/Math.PI) );
        }

        if (padStickLeftY < 0 && padStickLeftX > 0) {
            stickAngle += 360 ;
        } else if (padStickLeftY < 0 && padStickLeftX < 0) {
            stickAngle += 180 ;
        } else if (padStickLeftY > 0 && padStickLeftX < 0) {
            stickAngle += 180 ;
        }

        gamePad1LeftStickMagnitude = (float) Math.pow((padStickLeftX*padStickLeftX + padStickLeftY*padStickLeftY), 0.5) ;
        polarCoordinates[0] = gamePad1LeftStickMagnitude ;
        polarCoordinates[1] = stickAngle ;

        return polarCoordinates ;
    }

    public double maxPowerIdentifier (double motorPower00, double motorPower01, double motorPower10, double motorPower11) {
        double power1 = Math.max(motorPower00,motorPower01) ;
        double power2 = Math.max(motorPower10,motorPower11) ;
        return Math.max(power1,power2) ;
    }

}

/*
if (gamepad2.dpad_up) {
        polarCoordinates[0] = (float) 0.2 ;
        } else {
        polarCoordinates[0] = 0 ;
        }

        if (gamepad2.left_bumper && !headingToggle) {
        headingToggle = true ;
        OmniBot.resetFirstPIDDrive();
        targetHeading = targetHeading + 90;
        RobotLog.i("target heading is " + targetHeading);
        RobotLog.i("button pressed") ;
        } else if (!gamepad2.left_bumper && headingToggle) {
        headingToggle = false ;
        RobotLog.i("button released ") ;
        }

        if (gamepad2.right_bumper && !headingToggle) {
        headingToggle = true;
        OmniBot.resetFirstPIDDrive();
        targetHeading = targetHeading - 90;
        RobotLog.i("target heading is " + targetHeading);
        }else if (!gamepad2.right_bumper && headingToggle) {
        headingToggle = false ;
        }

        if (gamepad2.dpad_left && !directionToggle) {
        directionToggle = true ;
        OmniBot.resetFirstPIDDrive();
        polarCoordinates[1] = polarCoordinates[1] + 45;
        RobotLog.i("direction is " + polarCoordinates[1]);
        } else if (!gamepad2.dpad_left && directionToggle) {
        directionToggle = false ;
        }

        if (gamepad2.dpad_right && !directionToggle) {
        directionToggle = true ;
        OmniBot.resetFirstPIDDrive();
        polarCoordinates[1] = polarCoordinates[1] - 45 ;
        RobotLog.i("direction is " + polarCoordinates[1]);
        } else if (!gamepad2.dpad_right && directionToggle) {
        directionToggle = false ;
        }

        targetHeading = normalizeAngle(targetHeading) ;
        polarCoordinates[1] = (float) normalizeAngle((double) polarCoordinates[1]);

        RobotLog.i("target heading is " + targetHeading);

        OmniBot.driveOmniBot(polarCoordinates[0], polarCoordinates[1],  (float) targetHeading);
*/
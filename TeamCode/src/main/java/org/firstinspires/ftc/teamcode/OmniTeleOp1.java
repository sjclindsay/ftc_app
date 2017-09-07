package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.ftccommon.DbgLog;
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
    float leftStickY = 0 ;
    boolean controller1 = true;
    boolean controller2 = false ;
    float dPadScalar = 1 ;
    HardwareOmniBot OmniBot ;
    boolean waitForUpRelease = false ;
    boolean waitForDownRelease = false ;
    protected  float gamePad1LeftStickMagnitude = 0 ;
    protected  double maxPower = 1;
    public enum STICKHEADING {stationary, north, northEast, east, southEast,south, southWest, west, northWest}
    public STICKHEADING currentStickHeading = STICKHEADING.stationary ;
    double stickAngle = 0 ;
    int [] [] motorPowerMatrix = new int [2] [2] ;

    @Override
    public void init() {
        OmniBot = new HardwareOmniBot() ;
        OmniBot.init(hardwareMap);
        waitForUpRelease = false ;
        waitForUpRelease = false ;
        dPadScalar = 1 ;
        motorLeft1power = 0;
        motorLeft2power = 0;
        motorRight1power = 0;
        motorRight2power = 0;
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
        currentStickHeading =  getCurrentStickHeading(-gamepad2.left_stick_y, gamepad2.left_stick_x) ;

        if (gamepad1.a || controller1) {
            controller1 = true ;
            controller2 = false ;

            motorLeft1power = (leftStickY  + gamepad1.left_stick_x + gamepad1.right_stick_x)/dPadScalar ;
            motorLeft2power = (leftStickY  - gamepad1.left_stick_x + gamepad1.right_stick_x)/dPadScalar;
            motorRight1power = (leftStickY - gamepad1.left_stick_x - gamepad1.right_stick_x)/dPadScalar;
            motorRight2power = (leftStickY + gamepad1.left_stick_x - gamepad1.right_stick_x)/dPadScalar;

            OmniBot.setBotMovement(motorLeft1power, motorLeft2power, motorRight1power, motorRight2power);

        }
        if (gamepad2.a || controller2) {
            controller2 = true ;
            controller1 = false ;
            complexOmniBotMath(-gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x, dPadScalar, currentStickHeading);
        }

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
            dPadScalar += 1 ;
        } else if (!dPadUpValue && waitForUpRelease) {
            waitForUpRelease = false ;
        }
        if (dPadDownValue && !waitForDownRelease) {
            waitForDownRelease = true ;
            dPadScalar -= 1 ;
        } else if (!dPadDownValue && waitForDownRelease) {
            waitForDownRelease = false ;
        }
        return dPadScalar ;
    }

    public STICKHEADING getCurrentStickHeading (float padStickLeftY, float padStickLeftX) {
        stickAngle = Math.atan(padStickLeftY/padStickLeftX) ;

        if (padStickLeftY < 0 && padStickLeftX > 0) {
            stickAngle += 360 ;
        } else if (padStickLeftY < 0 && padStickLeftX < 0) {
            stickAngle += 180 ;
        } else if (padStickLeftY > 0 && padStickLeftX < 0) {
            stickAngle += 180 ;
        }

        if (padStickLeftY == 0 && padStickLeftX == 0) {currentStickHeading = STICKHEADING.stationary ;}
        else if (stickAngle <= 22.5 || stickAngle > 337.5 ) {currentStickHeading = STICKHEADING.east ;}
        else if ( stickAngle <= 67.5 && stickAngle > 22.5) {currentStickHeading = STICKHEADING.northEast ;}
        else if ( stickAngle <= 112.5 && stickAngle > 67.5) {currentStickHeading = STICKHEADING.north ;}
        else if ( stickAngle <= 157.5 && stickAngle > 112.5) {currentStickHeading = STICKHEADING.northWest ;}
        else if ( stickAngle <= 202.5 && stickAngle > 157.5) {currentStickHeading = STICKHEADING.west ;}
        else if ( stickAngle <= 247.5 && stickAngle > 202.5) {currentStickHeading = STICKHEADING.southWest ;}
        else if ( stickAngle <= 292.5 && stickAngle > 247.5) {currentStickHeading = STICKHEADING.south ;}
        else if ( stickAngle <= 337.5 && stickAngle > 292.5) {currentStickHeading = STICKHEADING.southEast ;}

        return  currentStickHeading ;
    }

    public void complexOmniBotMath (float padLeftStickY, float padLeftStickX, float padRightStickx, double dPadScalar, STICKHEADING currentStickHeading) {
        double motorPower00  ;
        double motorPower01 ;
        double motorPower10 ;
        double motorPower11 ;

        if (currentStickHeading == STICKHEADING.stationary) {
            motorPowerMatrix[0][0] = 0;
            motorPowerMatrix[0][1] = 0;
            motorPowerMatrix[1][0] = 0;
            motorPowerMatrix[1][1] = 0;
        } else if (currentStickHeading == STICKHEADING.north) {
            motorPowerMatrix[0][0] = 1;
            motorPowerMatrix[0][1] = 1;
            motorPowerMatrix[1][0] = 1;
            motorPowerMatrix[1][1] = 1;
        } else if (currentStickHeading == STICKHEADING.northEast) {
            motorPowerMatrix[0][0] = 1;
            motorPowerMatrix[0][1] = 0;
            motorPowerMatrix[1][0] = 0;
            motorPowerMatrix[1][1] = 1;
        } else if (currentStickHeading == STICKHEADING.east) {
            motorPowerMatrix[0][0] = 1;
            motorPowerMatrix[0][1] = -1;
            motorPowerMatrix[1][0] = -1;
            motorPowerMatrix[1][1] = 1;
        } else if (currentStickHeading == STICKHEADING.southEast) {
            motorPowerMatrix[0][0] = 0;
            motorPowerMatrix[0][1] = -1;
            motorPowerMatrix[1][0] = -1;
            motorPowerMatrix[1][1] = 0;
        } else if (currentStickHeading == STICKHEADING.south) {
            motorPowerMatrix[0][0] = -1;
            motorPowerMatrix[0][1] = -1;
            motorPowerMatrix[1][0] = -1;
            motorPowerMatrix[1][1] = -1;
        } else if (currentStickHeading == STICKHEADING.southWest) {
            motorPowerMatrix[0][0] = -1;
            motorPowerMatrix[0][1] = 0;
            motorPowerMatrix[1][0] = 0;
            motorPowerMatrix[1][1] = -1;
        } else if (currentStickHeading == STICKHEADING.west) {
            motorPowerMatrix[0][0] = -1;
            motorPowerMatrix[0][1] = 1;
            motorPowerMatrix[1][0] = 1;
            motorPowerMatrix[1][1] = -1;
        } else if (currentStickHeading == STICKHEADING.northWest) {
            motorPowerMatrix[0][0] = 0;
            motorPowerMatrix[0][1] = 1;
            motorPowerMatrix[1][0] = 1;
            motorPowerMatrix[1][1] = 0;
        }

        gamePad1LeftStickMagnitude = (float) Math.pow((padLeftStickX*padLeftStickX +padLeftStickY*padLeftStickY), 0.5) ;

        motorPower00 = gamePad1LeftStickMagnitude*motorPowerMatrix[0][0] ;
        motorPower01 = gamePad1LeftStickMagnitude*motorPowerMatrix[0][1] ;
        motorPower10 = gamePad1LeftStickMagnitude*motorPowerMatrix[1][0] ;
        motorPower11 = gamePad1LeftStickMagnitude*motorPowerMatrix[1][1] ;

        motorPower00 = motorPower00 - padRightStickx ;
        motorPower01 = motorPower01 - padRightStickx ;
        motorPower10 = motorPower10 + padRightStickx ;
        motorPower11 = motorPower11 + padRightStickx ;

        if (motorPower00 > 1 || motorPower01 > 1 || motorPower10 > 1 || motorPower11 > 1) {
            maxPower = maxPowerIdentifier(motorPower00, motorPower01, motorPower10, motorPower11) ;
            motorPower00 = motorPower00/(maxPower*dPadScalar) ;
            motorPower01 = motorPower10/(maxPower*dPadScalar) ;
            motorPower10 = motorPower10/(maxPower*dPadScalar) ;
            motorPower11 = motorPower11/(maxPower*dPadScalar) ;
        }

        OmniBot.setBotMovement(motorPower00, motorPower01, motorPower10, motorPower11);
    }

    public double maxPowerIdentifier (double motorPower00, double motorPower01, double motorPower10, double motorPower11) {
        double power1 = Math.max(motorPower00,motorPower01) ;
        double power2 = Math.max(motorPower10,motorPower11) ;
        return Math.max(power1,power2) ;
    }

}

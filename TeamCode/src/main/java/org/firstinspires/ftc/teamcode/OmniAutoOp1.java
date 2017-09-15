package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by conno on 8/17/2017.
 */

@TeleOp(name="Omni: AutoOp1", group="Omni")


public class OmniAutoOp1 extends OpMode {
    public enum MotorState{
        ERROR_STATE,
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

    float motorLeft1power = 0;
    float motorLeft2power = 0;
    float motorRight1power = 0;
    float motorRight2power = 0;
    float leftStickY = 0 ;
    boolean controller1 = true;
    boolean controller2 = false ;
    robotHWconnected autoConnectedHW = robotHWconnected.MotorGyro;

    HardwareOmniBot OmniBot ;

    @Override
    public void init() {
        OmniBot = new HardwareOmniBot(autoConnectedHW) ;
        OmniBot.init(hardwareMap);
        composeTelemetry();
    }

    @Override
    public void start() {

        OmniBot.start();
    }

    @Override
    public void loop() {
        leftStickY = -gamepad1.left_stick_y ;

        if (gamepad1.a || controller1) {
            controller1 = true ;
            controller2 = false ;

            motorLeft1power = leftStickY  + gamepad1.left_stick_x + gamepad1.right_stick_x;
            motorLeft2power = leftStickY  - gamepad1.left_stick_x + gamepad1.right_stick_x;
            motorRight1power = leftStickY - gamepad1.left_stick_x - gamepad1.right_stick_x;
            motorRight2power = leftStickY + gamepad1.left_stick_x - gamepad1.right_stick_x;

            OmniBot.setBotMovement(motorLeft1power, motorLeft2power, motorRight1power, motorRight2power);

        }
        if (gamepad2.a || controller2) {
            controller2 = true ;
            controller1 = false ;

            OmniBot.driveOmniBot( (float) 0.1, 0, 0);
        }

        OmniBot.waitForTick(40);
        telemetry.update();

    }

    @Override
    public void stop() {

    }
    void composeTelemetry() {
        OmniBot.addTelemetry(telemetry);
        //OmniBot.getTelemetry(telemetry);
    }




}

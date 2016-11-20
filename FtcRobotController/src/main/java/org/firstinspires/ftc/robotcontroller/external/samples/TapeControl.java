package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This is all the functions and control for the Tape Extenders
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 */
public class TapeControl
{
    /* Public OpMode members. */
    public CRServo leftTape   = null;
    public CRServo rightTape  = null;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;

    /* Constructor */
    public TapeControl() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftTape   = hwMap.crservo.get("servoTapeLeft");
        rightTape  = hwMap.crservo.get("servoTapeRight");
        rightTape.setDirection(CRServo.Direction.REVERSE);

        // Set all motors to zero power
        leftTape.setPower(0);
        rightTape.setPower(0);

        // Define and initialize ALL installed servos.
        leftTape.setPower(0.0);
        rightTape.setPower(0.0);
    }

}

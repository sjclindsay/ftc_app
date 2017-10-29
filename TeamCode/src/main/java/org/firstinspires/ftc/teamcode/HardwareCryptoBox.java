package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by conno on 10/26/2017.
 */

public class HardwareCryptoBox {

    DigitalChannel cryptoBoxTouch1 = null ;
    DigitalChannel cryptoBoxTouch2 = null ;
    Servo cryptoBoxServo = null ;
    protected boolean cryptoBoxTouchValue1 = false ;
    protected boolean cryptoBoxTouchValue2 = false ;
    protected float cryptoBoxServoPositionUp = (float) 0.5 ;
    protected float cryptoBoxServoPositionDown = 0 ;
    HardwareMap hwMap = null ;

    public  HardwareCryptoBox() {

    }

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        cryptoBoxTouch1 = hwMap.digitalChannel.get("cryptoBoxTouch1") ;
        cryptoBoxTouch2 = hwMap.digitalChannel.get("cryptoBoxTouch2") ;
        cryptoBoxServo = hwMap.servo.get("cryptoBoxServo") ;

        cryptoBoxTouch1.setMode(DigitalChannel.Mode.INPUT);
        cryptoBoxTouch2.setMode(DigitalChannel.Mode.INPUT);
        cryptoBoxServo.setPosition(cryptoBoxServoPositionUp);
    }

    public void start() {

    }

    public void raiseCryptoServo() {
        cryptoBoxServo.setPosition(cryptoBoxServoPositionUp);
    }
    public void lowerCryptoServo() {
        cryptoBoxServo.setPosition(cryptoBoxServoPositionDown);
    }

    public boolean updateCryptoTouch1() {
        return cryptoBoxTouch1.getState();
    }
    public boolean updateCryptoTouch2() {
        return cryptoBoxTouch2.getState();
    }


}

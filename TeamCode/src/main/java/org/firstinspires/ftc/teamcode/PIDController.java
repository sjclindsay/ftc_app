package org.firstinspires.ftc.teamcode;

/**
 * This is NOT an opmode.
 *
 * THis Class is used to define a PID controller of any kind
 *
 */

public class PIDController
{
    private double lastError_;
    private double setPoint_;
    private double errorSum_;
    private double kp_;
    private double ki_;
    private double kd_;
    private long lastTime_;

    /* Public OpMode members. */
    private PIDController() {}

    /* Constructor */
    public PIDController(double setPoint){
        setPoint_ = setPoint;
        lastError_ = 0;
        lastTime_ = System.currentTimeMillis();
        errorSum_ = 0;

        kp_ = (float)0.001 ;
        ki_ = (float)0;
        kd_ = (float)0;
    }

    public PIDController(double setPoint, double kp, double ki, double kd){
        setPoint_ = setPoint;
        lastError_ = 0;
        lastTime_ = System.currentTimeMillis();
        errorSum_ = 0;

        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    public double Update(double newInput){

        long time = System.currentTimeMillis();
        long period = time - lastTime_;
        double error  = setPoint_ - newInput;
        errorSum_ += (error * period);
        double derError = 0; //(error - lastError_) / period;

        double output = (kp_ * error) + (ki_ * errorSum_) + (kd_ * derError);

        lastError_ = error;
        lastTime_ = time;
        return output;
    }
}


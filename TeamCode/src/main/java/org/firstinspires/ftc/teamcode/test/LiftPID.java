package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LiftPID {

    public static double previous_error = 0.0; //error before current error
    public static double PID_p_error = 0.0; //kp * error
    public static double PID_i_error = 0.0; //Ki * error + Previous Integral correction
    public static double PID_d_error = 0.0; //kd * ((prev. error - current error) / ElapsedTime)
    public static double PID_total = 0.0;

    public static double kp = 1.0; //affects sensitivity for proportional controller
    public static double ki = 0.005; //affects sensitivity for integral controller
    public static double kd = 30; //affects sensitivity for derivative controller

    public double getTargetVelocity(double error) {
        //timer
        ElapsedTime timer_1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer_1.startTime();

        PID_p_error = kp * error;
        PID_i_error = ki * error + PID_i_error;
        PID_d_error = kd * (error - previous_error) / timer_1.milliseconds();
        // PID_total = PID_p_error + PID_i_error + PID_d_error;
        PID_total = PID_p_error ;

        previous_error = error; //the current error for this round will be previous error for next cycle
        timer_1.reset(); //resetting elapsed time
        return myMap(PID_total, 20.0,900.0, 0.0, 0.5);
        }

    //to normalize values between in range and out range
    public double myMap(double value, double inLow, double inHigh, double outLow, double outHigh) {
        return outLow + (outHigh-outLow)*(value - inLow)/(inHigh-inLow);
    }
}
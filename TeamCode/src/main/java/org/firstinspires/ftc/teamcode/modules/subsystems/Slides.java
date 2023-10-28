package org.firstinspires.ftc.teamcode.modules.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.statemachines.SlidesSM;

public class Slides extends Subsystem {

    public SlidesSM statemachine;

    public DcMotor leftMotor;
    public DcMotor rightMotor;

    double leftMotorPos;
    double rightMotorPos;

    //PID STUFF
    double integralSum = 0;
    double kp = 0;
    double ki = 0;
    double kd = 0;
    double error;
    private double lasterror = 0;
    double reference; //the point you want to go to
    double state;

    ElapsedTime timer = new ElapsedTime();



    public Slides(HardwareMap hardwareMap) {

        leftMotor = hardwareMap.get(DcMotor.class, "leftLift");
        rightMotor = hardwareMap.get(DcMotor.class, "rightLight");

        statemachine = new SlidesSM(this);
    }

    public void setLiftPosition(int pos) {
        leftMotor.setTargetPosition(pos);
        rightMotor.setTargetPosition(pos);
    }

    public double PIDControl(double reference,double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error- lasterror) / timer.seconds();
        lasterror = error;

        timer.reset();

        double output = (error * kp) + (derivative * kd) + (integralSum * ki);
        return output;
    }

    @Override
    public void update() {
        leftMotorPos = leftMotor.getCurrentPosition();
        rightMotorPos = rightMotor.getCurrentPosition();

        

        statemachine.update();
    }
}

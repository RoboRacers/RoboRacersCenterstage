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

    double location;

    double averagePos;

    //PID STUFF
    double integralSum = 0;
    double kp = 0;
    double ki = 0;
    double kd = 0;
    private double lasterror = 0;

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

    public double motorPos(){
        leftMotorPos = leftMotor.getCurrentPosition();
        rightMotorPos = rightMotor.getCurrentPosition();
        averagePos = (leftMotorPos + rightMotorPos)/2;
        return averagePos;
    }

    public void PIDLift(int location){
        this.location = location;
    }

    @Override
    public void update() {
        statemachine.update();

        if ((int)(motorPos()) != location){
            setLiftPosition((int)(PIDControl(location, motorPos())));
        }

    }
}

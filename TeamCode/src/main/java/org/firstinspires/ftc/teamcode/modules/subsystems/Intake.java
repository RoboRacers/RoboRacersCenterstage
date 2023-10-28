package org.firstinspires.ftc.teamcode.modules.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.statemachines.IntakeSM;
import org.firstinspires.ftc.teamcode.modules.statemachines.StateMachine;

public class Intake extends Subsystem {

    public IntakeSM statemachine;

    public Servo claw;
    public Servo claw_flip;
    public Servo claw_extend_one;
    //public Servo claw_extend_two;

    public DcMotor leftMotor;
    public DcMotor rightMotor;

    double currentSpeed;

    public Intake(HardwareMap hardwareMap) {
        //4 servos

        claw = hardwareMap.get(Servo.class, "claw");
        claw_flip = hardwareMap.get(Servo.class, "claw_flip");
        claw_extend_one = hardwareMap.get(Servo.class, "claw_extend_one");
       // claw_extend_two = hardwareMap.get(Servo.class, "claw_extend_two");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        statemachine = new IntakeSM(this);
    }

    public void setClawPos(double pos1) {claw.setPosition(pos1);}

    public void setIntakePos(double pos2, double pos3, double speed1, int motorPos, double speed2, int motorPos2, DcMotorSimple.Direction direction) {
        claw_flip.setPosition(pos2);
        claw_extend_one.setPosition(pos3);
        leftMotor.setPower(speed1);
        rightMotor.setTargetPosition(motorPos);
        rightMotor.setPower(speed2);
        leftMotor.setTargetPosition(motorPos2);
        leftMotor.setDirection(direction);
    }

     //public void setClawExtendTwoPos(int pos4) {claw_extend_two.setPosition(pos4);}
    //public void setClawExtendTwoDirection(Servo.Direction direction){claw_extend_two.setDirection(direction);}

    @Override
    public void update() {
        statemachine.update();
    }
}

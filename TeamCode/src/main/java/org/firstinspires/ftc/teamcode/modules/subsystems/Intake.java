package org.firstinspires.ftc.teamcode.modules.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.statemachines.IntakeSM;
import org.firstinspires.ftc.teamcode.modules.statemachines.StateMachine;

public class Intake extends Subsystem {
    double closed = 0.25;
    double open = 0;
    double out = 0.5;
    double in = 0;
    double extend = 0.25;
    double retract = 0;
    public IntakeSM statemachine;

    public Servo claw;
    public Servo stage1Right;
    public Servo stage1Left;
    public Servo stage2Right;
    public Servo stage2Left;

    public Intake(HardwareMap hardwareMap) {
        //4 servos
        stage1Right = hardwareMap.get(Servo.class, "stage1right");
        stage1Left = hardwareMap.get(Servo.class, "stage1left");
        stage2Right = hardwareMap.get(Servo.class, "stage2right");
        stage2Left = hardwareMap.get(Servo.class, "stage2left");
        claw = hardwareMap.get(Servo.class, "claw");

        stage1Left.setDirection(Servo.Direction.REVERSE);
        stage2Left.setDirection(Servo.Direction.REVERSE);
        claw.setDirection(Servo.Direction.REVERSE);

        statemachine = new IntakeSM(this);
    }

    public void setClawPos(double pos1) {claw.setPosition(pos1);}


    public void setIntake(double clawpos, double s1, double s2){
        claw.setPosition(clawpos);
        stage1Right.setPosition(s1);
        stage1Left.setPosition(s1+.1);
        stage2Right.setPosition(s2);
        stage2Right.setPosition(s2);
    }

    public void intakeStageOne() {
        claw.setPosition(0.3);
        stage1Right.setPosition(0.7);
        stage1Left.setPosition(0.7);
        stage2Right.setPosition(0.8);
        stage2Right.setPosition(0.8);
    }

    public void intakeStageTwo() {
        claw.setPosition(0.6);
        stage1Right.setPosition(0.75);
        stage1Left.setPosition(0.75);
        stage2Right.setPosition(0.8);
        stage2Right.setPosition(0.8);
    }

    public void intakeStageThree() {
        claw.setPosition(0.2);
        stage1Right.setPosition(0.73);
        stage1Left.setPosition(0.73);
        stage2Right.setPosition(0.8);
        stage2Right.setPosition(0.8);
    }

    @Override
    public void update() {
        statemachine.update();
    }
}

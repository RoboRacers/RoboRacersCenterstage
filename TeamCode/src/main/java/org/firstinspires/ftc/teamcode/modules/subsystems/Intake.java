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
    public Servo claw_flip_one;
    public Servo claw_flip_two;
    public Servo claw_extend_one;
    public Servo claw_extend_two;

    public DcMotor leftMotor;
    public DcMotor rightMotor;

    double currentSpeed;

    public Intake(HardwareMap hardwareMap) {
        //4 servos

        claw = hardwareMap.get(Servo.class, "claw");
        claw_flip_one = hardwareMap.get(Servo.class, "claw_flip_one");
        claw_flip_two = hardwareMap.get(Servo.class, "claw_flip_two");
        claw_extend_one = hardwareMap.get(Servo.class, "claw_extend_one");
        claw_extend_two = hardwareMap.get(Servo.class, "claw_extend_two");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        statemachine = new IntakeSM(this);
    }

    public void setClawPos(double pos1) {claw.setPosition(pos1);}

    /*
    public void setIntakePos(double pos2, double pos3, double speed1, int motorPos, double speed2, int motorPos2, DcMotorSimple.Direction direction) {
        claw_flip.setPosition(pos2);
        claw_extend_one.setPosition(pos3);
        leftMotor.setPower(speed1);
        rightMotor.setTargetPosition(motorPos);
        rightMotor.setPower(speed2);
        leftMotor.setTargetPosition(motorPos2);
        leftMotor.setDirection(direction);
    }
    */

    public void setIntake(boolean intakeExtend){
        if (intakeExtend=true){
            claw_flip_one.setPosition(out);
            claw_flip_two.setDirection(Servo.Direction.REVERSE);
            claw_flip_two.setPosition(out);
            claw_extend_one.setPosition(extend);
            claw_extend_two.setPosition(extend);
            claw_extend_two.setDirection(Servo.Direction.REVERSE);
          //  leftMotor.setPower(500);
         //   rightMotor.setTargetPosition(5);
         //   rightMotor.setPower(500);
         //   leftMotor.setTargetPosition(5);
         //   leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (intakeExtend=false){
            claw_flip_one.setPosition(in);
            claw_flip_two.setDirection(Servo.Direction.REVERSE);
            claw_flip_two.setPosition(in);
            claw_extend_one.setPosition(retract);
            claw_extend_two.setDirection(Servo.Direction.REVERSE);
            claw_extend_two.setPosition(retract);
         //   leftMotor.setPower(500);
           // rightMotor.setTargetPosition(0);
           // rightMotor.setPower(500);
        //    leftMotor.setTargetPosition(0);
          //  leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }
     //public void setClawExtendTwoPos(int pos4) {claw_extend_two.setPosition(pos4);}
    //public void setClawExtendTwoDirection(Servo.Direction direction){claw_extend_two.setDirection(direction);}

    @Override
    public void update() {
        statemachine.update();
    }
}

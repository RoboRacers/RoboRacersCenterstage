package org.firstinspires.ftc.teamcode.modules.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.statemachines.IntakeSM;
import org.firstinspires.ftc.teamcode.modules.statemachines.StateMachine;

public class Intake extends Subsystem {

    public IntakeSM statemachine;

    public DcMotor intakeMotor;

    public Servo claw;
    public Servo claw_flip;
    public Servo claw_extend_one;
    public Servo claw_extend_two;

    double currentSpeed;

    public Intake(HardwareMap hardwareMap) {
        //4 servos

        claw = hardwareMap.get(Servo.class, "claw");
        claw_flip = hardwareMap.get(Servo.class, "claw_flip");
        claw_extend_one = hardwareMap.get(Servo.class, "claw_extend_one");
        claw_extend_two = hardwareMap.get(Servo.class, "claw_extend_two");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");


        statemachine = new IntakeSM(this);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.setPower(currentSpeed);
    }

    @Override
    public void update() {
        statemachine.update();
    }


}

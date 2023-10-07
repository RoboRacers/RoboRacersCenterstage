package org.firstinspires.ftc.teamcode.modules.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.statemachines.DepositSM;

public class Deposit extends Subsystem {


    public DcMotor leftMotor;
    public DcMotor rightMotor;

    double leftMotorPos;
    double rightMotorPos;

    public DepositSM statemachine;


    public Deposit(HardwareMap hardwareMap) {

        leftMotor = hardwareMap.get(DcMotor.class, "leftLift");
        rightMotor = hardwareMap.get(DcMotor.class, "rightLight");

        statemachine = new DepositSM(this);
    }

    public void setLiftPosition(int pos) {
        leftMotor.setTargetPosition(pos);
        leftMotor.setTargetPosition(pos);
    }

    @Override
    public void update() {
        leftMotorPos = leftMotor.getCurrentPosition();
        rightMotorPos = rightMotor.getCurrentPosition();

        statemachine.update();
    }
}

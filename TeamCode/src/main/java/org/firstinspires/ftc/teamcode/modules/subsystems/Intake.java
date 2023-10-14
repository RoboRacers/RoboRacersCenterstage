package org.firstinspires.ftc.teamcode.modules.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.statemachines.IntakeSM;
import org.firstinspires.ftc.teamcode.modules.statemachines.StateMachine;

public class Intake extends Subsystem {

    public IntakeSM statemachine;

    public DcMotor intakeMotor;

    double currentSpeed;

    public Intake(HardwareMap hardwareMap) {

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

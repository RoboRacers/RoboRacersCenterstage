package org.firstinspires.ftc.teamcode.modules.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.statemachines.LauncherSM;

public class Launcher extends Subsystem  {


    public LauncherSM statemachine;

    public CRServo actuationServo;

    public Launcher(HardwareMap hardwareMap) {
        actuationServo = hardwareMap.get(CRServo.class, "actuationServo");
        statemachine = new LauncherSM(this);
    }

    public void setServoSpeed(double speed){actuationServo.setPower(speed);}

    @Override
    public void update() {statemachine.update();}
}

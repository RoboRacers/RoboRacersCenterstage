package org.firstinspires.ftc.teamcode.modules.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.statemachines.LauncherSM;

public class Launcher extends Subsystem  {

    public LauncherSM statemachine;

    public Servo actuationServo;

    public Launcher(HardwareMap hardwareMap) {
        actuationServo = hardwareMap.get(Servo.class, "actuationServo");
        statemachine = new LauncherSM(this);
    }

    public void setServoPos(boolean ifshoot){
        if(ifshoot){
            actuationServo.setPosition(1);
        } else {
            actuationServo.setPosition(0);
        }
    }

    @Override
    public void update() {
        statemachine.update();
    }
}

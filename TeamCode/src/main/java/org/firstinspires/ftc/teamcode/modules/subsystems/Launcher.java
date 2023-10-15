package org.firstinspires.ftc.teamcode.modules.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.statemachines.LauncherSM;

public class Launcher extends Subsystem  {

    LauncherSM statemachine;

    Servo actuationServo;

    public Launcher(HardwareMap hardwareMap) {
        actuationServo = hardwareMap.get(Servo.class, "actuationServo");
        statemachine = new LauncherSM(this);
    }

    @Override
    public void update() {
        statemachine.update();
    }
}

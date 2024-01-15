package org.firstinspires.ftc.teamcode.modules.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher implements Subsystem  {


    public Servo actuationServo;

    public Launcher(HardwareMap hardwareMap) {
        actuationServo = hardwareMap.get(Servo.class, "actuationServo");

    }

    public void fireDrone(boolean ifshoot){
        if(ifshoot){
            actuationServo.setPosition(1);
        } else {
            actuationServo.setPosition(0);
        }
    }

    @Override
    public void update() {
    }
}

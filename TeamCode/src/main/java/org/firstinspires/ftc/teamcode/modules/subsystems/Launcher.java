package org.firstinspires.ftc.teamcode.modules.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Launcher implements Subsystem  {


    public ServoImplEx actuationServo;

    public Launcher(HardwareMap hardwareMap) {
        actuationServo = hardwareMap.get(ServoImplEx.class, "actuationServo");

    }

    public void fireDrone(boolean ifshoot){
        if(ifshoot){
            actuationServo.setPosition(0.45);
        } else {
            actuationServo.setPosition(0.5);
        }
    }

    @Override
    public void update() {
    }
}

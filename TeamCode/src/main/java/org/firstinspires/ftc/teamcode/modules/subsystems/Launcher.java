package org.firstinspires.ftc.teamcode.modules.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Launcher implements Subsystem  {


    public ServoImplEx actuationServo;

    public static double firePos = 0.17;

    public Launcher(HardwareMap hardwareMap) {
        actuationServo = hardwareMap.get(ServoImplEx.class, "actuationServo");
    }

    public void fireDrone(boolean ifshoot){
        if(ifshoot){
            actuationServo.setPosition(firePos);
        } else {
            actuationServo.setPosition(0.43);
        }
    }

    @Override
    public void update() {
    }
}

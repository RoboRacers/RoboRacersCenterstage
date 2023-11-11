package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.statemachines.IntakeSM;
import org.firstinspires.ftc.teamcode.modules.statemachines.LauncherSM;
import org.firstinspires.ftc.teamcode.modules.subsystems.Intake;
import org.firstinspires.ftc.teamcode.modules.subsystems.Launcher;

@TeleOp(name = "Launcher_TEST", group = "16481-Power-Play")
public class TeleopTESTING extends LinearOpMode {

//Servo drone;
//DcMotorEx slides1;
//DcMotorEx slides2;

Servo Fs1;
Servo Fs2;
Servo Ss1;
Servo Ss2;
ElapsedTime timer = new ElapsedTime();

double MINPOS = 0.0;
double MAXPOS = 1.0;


   @Override
    public void runOpMode(){


       // drone = hardwareMap.get(Servo.class, "drone");
     //   slides1 = hardwareMap.get(DcMotorEx.class, "slides1");
      //  slides2 = hardwareMap.get(DcMotorEx.class, "slides2");
      //  slides2.setDirection(DcMotorSimple.Direction.REVERSE);
        Fs1 = hardwareMap.get(Servo.class, "fs1");
        Fs2 = hardwareMap.get(Servo.class, "fs2");
        Ss1 = hardwareMap.get(Servo.class, "Ss1");
Ss2 = hardwareMap.get(Servo.class, "Ss2");

        while (opModeInInit()){
            Fs2.setDirection(Servo.Direction.REVERSE);
            Ss2.setDirection(Servo.Direction.REVERSE);
            }
        while (opModeIsActive()){

                Fs1.setPosition(0.25);

        }
telemetry.update();
    }
    public void set1(double pos1){
        Fs1.setPosition(pos1);
        Fs2.setPosition(pos1);
    }

}

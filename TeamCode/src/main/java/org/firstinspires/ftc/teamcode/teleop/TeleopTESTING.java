package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Launcher_TEST", group = "16481-Power-Play")
public class TeleopTESTING extends LinearOpMode {

//Servo drone;
//DcMotorEx slides1;
//DcMotorEx slides2;

Servo Bs1;
Servo Bs2;
Servo Ss1;
Servo Ss2;
Servo Cs1;
ElapsedTime timer = new ElapsedTime();

double MINPOS = 0.0;
double MAXPOS = 1.0;


   @Override
    public void runOpMode(){


       // drone = hardwareMap.get(Servo.class, "drone");
     //   slides1 = hardwareMap.get(DcMotorEx.class, "slides1");
      //  slides2 = hardwareMap.get(DcMotorEx.class, "slides2");
      //  slides2.setDirection(DcMotorSimple.Direction.REVERSE);
        Bs1 = hardwareMap.get(Servo.class, "fs1");
        Bs2 = hardwareMap.get(Servo.class, "fs2");
        Ss1 = hardwareMap.get(Servo.class, "Ss1");
        Ss2 = hardwareMap.get(Servo.class, "Ss2");
        Cs1 = hardwareMap.get(Servo.class, "Cs1");

        while (opModeInInit()){
            Bs2.setDirection(Servo.Direction.REVERSE);
            Ss2.setDirection(Servo.Direction.REVERSE);
            }
        while (opModeIsActive()){
            if(gamepad1.dpad_up){

            }

                Bs1.setPosition(0.25);

        }
telemetry.update();
    }
    public void set1(double pos1){
        Bs1.setPosition(pos1);
        Bs2.setPosition(pos1);
    }

}

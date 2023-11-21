package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Intake Range Test", group = "16481-Power-Play")
public class IntakeRangeTest extends LinearOpMode {

//Servo drone;
//DcMotorEx slides1;
//DcMotorEx slides2;

Servo Bs1;
Servo Bs2;
Servo Ss1;
Servo Ss2;
Servo Cs1;
ElapsedTime timer = new ElapsedTime();

double Increment = 0.001;
double MINPOS = 0.0;
double MAXPOS = 1.0;
double position1 = 0.5;
double position2 = 0.6;
double position3 = 0.5;
double position4 = 0.5;
double position5 = 0.0;



   @Override
    public void runOpMode(){


       // drone = hardwareMap.get(Servo.class, "drone");
     //   slides1 = hardwareMap.get(DcMotorEx.class, "slides1");
      //  slides2 = hardwareMap.get(DcMotorEx.class, "slides2");
      //  slides2.setDirection(DcMotorSimple.Direction.REVERSE);
        Bs1 = hardwareMap.get(Servo.class, "stage1right");
        Bs2 = hardwareMap.get(Servo.class, "stage1left");
        Ss1 = hardwareMap.get(Servo.class, "stage2right");
        Ss2 = hardwareMap.get(Servo.class, "stage2left");
        Cs1 = hardwareMap.get(Servo.class, "claw");

        while (opModeInInit()){
            Bs2.setDirection(Servo.Direction.REVERSE);
            Ss2.setDirection(Servo.Direction.REVERSE);
            Cs1.setDirection(Servo.Direction.REVERSE);
            }
        while (opModeIsActive()){
            if(gamepad1.dpad_up){
                position1 += Increment;
                position2 += Increment;
                if (position2 >= MAXPOS){
                    position2 = MAXPOS;
                    position1 = MAXPOS - 0.1;
                }
            }
            else if(gamepad1.dpad_down){
                position1 -= Increment;
                position2 -= Increment;
                if (position1 <= MINPOS){
                    position1 = MINPOS;
                    position2 = MINPOS + 0.1;
                }
            }
            if(gamepad1.dpad_left){
                position3 += Increment;
                position4 += Increment;
                if (position3 >= MAXPOS){
                    position3 = MAXPOS;
                    position4 = MAXPOS;
                }
            }
            else if(gamepad1.dpad_right){
                position3 -= Increment;
                position4 -= Increment;
                if (position4 <= MINPOS){
                    position3 = MINPOS;
                    position4 = MINPOS;
                }
            }
            if (gamepad1.left_bumper){
                position5 = 0.54;
            }else if(gamepad1.right_bumper){
                position5 = 0.8;
            }
            
            set1(position1, position2, position3, position4, position5);
            telemetry.addData("Bs1 Value", Bs1.getPosition());
            telemetry.addData("Bs2 Value", Bs2.getPosition());
            telemetry.addData("Ss1 Value", Ss1.getPosition());
            telemetry.addData("Ss1 Value", Ss2.getPosition());
            telemetry.addData("Cs1 Value", Cs1.getPosition());
            telemetry.update();

        }
telemetry.update();
    }
    public void set1(double pos1, double pos2, double pos3, double pos4, double pos5){
        Bs1.setPosition(pos1);
        Bs2.setPosition(pos2);
        Ss1.setPosition(pos3);
        Ss2.setPosition(pos4);
        Cs1.setPosition(pos5);
    }

}
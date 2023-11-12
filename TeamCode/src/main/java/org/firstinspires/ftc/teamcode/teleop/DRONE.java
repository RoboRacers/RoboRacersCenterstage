package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "DRONE_TEST", group = "16481-Power-Play")
public class DRONE extends LinearOpMode {

Servo drone;
double Increment = 0.0002;
double position = 0.527;


ElapsedTime timer = new ElapsedTime();
   @Override
    public void runOpMode(){


        drone = hardwareMap.get(Servo.class, "drone");



        while (opModeInInit()){
            drone.setDirection(Servo.Direction.REVERSE);
            }
        while (opModeIsActive()){

           if (gamepad1.circle){
               drone.setPosition(0);
               }
               //telemetry.addData("shots fired", "");
           }
           telemetry.update();
        }
    }
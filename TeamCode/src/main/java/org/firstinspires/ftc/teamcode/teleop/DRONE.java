package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "DRONE_TEST", group = "16481-Power-Play")
public class DRONE extends LinearOpMode {

Servo drone;


ElapsedTime timer = new ElapsedTime();
   @Override
    public void runOpMode(){


        drone = hardwareMap.get(Servo.class, "drone");



        while (opModeInInit()){

            }
        while (opModeIsActive()){

           if (gamepad1.left_bumper){
             drone.setPosition(0.56);
               }
           else if (gamepad1.right_bumper) {
               drone.setPosition(0.9);
           }
            telemetry.addData("Drone position", drone.getPosition());
            telemetry.update();
           }
        }
    }
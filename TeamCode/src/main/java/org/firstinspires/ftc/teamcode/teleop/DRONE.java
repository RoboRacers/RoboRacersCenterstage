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

CRServo drone;


ElapsedTime timer = new ElapsedTime();
   @Override
    public void runOpMode(){


        drone = hardwareMap.get(CRServo.class, "drone");



        while (opModeInInit()){

            }
        while (opModeIsActive()){

           if (gamepad1.left_bumper){
               drone.setPower(0.001);
               timer.startTime();
               if (timer.time() >= 5){
                   drone.setPower(0.0);
               }
               timer.reset();
               }
               //telemetry.addData("shots fired", "");
           }
           telemetry.update();
        }
    }
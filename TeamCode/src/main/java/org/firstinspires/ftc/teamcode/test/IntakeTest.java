package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Intake Test", group = "16481-Power-Play")
public class IntakeTest extends LinearOpMode {

Servo stage1Right;
Servo stage1Left;
Servo stage2Right;
Servo stage2Left;
Servo claw;


public static double stage1pos = 0.5;

public static double stage2pos = 0.5;

public static double clawPos = 0.0;

    @Override
    public void runOpMode(){

        /**
         * S1: .4
         * S2: .2
         *
         * S1: .8
         * S2: .8
         */


        stage1Right = hardwareMap.get(Servo.class, "stage1right");
        stage1Left = hardwareMap.get(Servo.class, "stage1left");
        stage2Right = hardwareMap.get(Servo.class, "stage2right");
        stage2Left = hardwareMap.get(Servo.class, "stage2left");
        claw = hardwareMap.get(Servo.class, "claw");

       stage1Left.setDirection(Servo.Direction.REVERSE);
       stage2Left.setDirection(Servo.Direction.REVERSE);
       claw.setDirection(Servo.Direction.REVERSE);

        while (opModeInInit()){

        }

        while (opModeIsActive()){

            stage1Right.setPosition(stage1pos);
            stage1Left.setPosition(stage1pos + .1);
            stage2Right.setPosition(stage2pos);
            stage2Left.setPosition(stage2pos);
            claw.setPosition(clawPos);
            

            telemetry.addData("Stage 1 Right Value", stage1Right.getPosition());
            telemetry.addData("Stage 1 Left Value", stage1Left.getPosition());
            telemetry.addData("Stage 2 Right Value", stage2Right.getPosition());
            telemetry.addData("Stage 2 Left Value", stage2Left.getPosition());
            telemetry.addData("Claw Value", claw.getPosition());
            telemetry.update();

        }
    }

}

package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name = "Indiviudal Intake Test", group = "Test")
public class IntakeIndiviudalTest extends LinearOpMode {

    ServoImplEx stage1Right;
    ServoImplEx stage1Left;
    ServoImplEx stage2Right;
    ServoImplEx stage2Left;
    ServoImplEx claw;


    public static double rightstage1pos = 0.5;
    public static double leftstage1pos = 0.5;

    public static double rightstage2pos = 0.5;

    public static double leftstage2pos = 0.5;

    public static double clawPos = 0.0;

    public static int enableleft = 1;

    public static int enableright = 1;

    @Override
    public void runOpMode(){

        /**
         * S1: .4
         * S2: .2
         *
         * S1: .8
         * S2: .8
         */


        stage1Right = hardwareMap.get(ServoImplEx.class, "stage1right");
        stage1Left = hardwareMap.get(ServoImplEx.class, "stage1left");
        stage2Right = hardwareMap.get(ServoImplEx.class, "stage2right");
        stage2Left = hardwareMap.get(ServoImplEx.class, "stage2left");
        claw = hardwareMap.get(ServoImplEx.class, "claw");

       stage1Left.setDirection(Servo.Direction.REVERSE);
       stage2Left.setDirection(Servo.Direction.REVERSE);
       claw.setDirection(Servo.Direction.REVERSE);

        while (opModeInInit()){

        }

        while (opModeIsActive()){

            if (enableleft == 0) {
                stage1Left.setPwmDisable();
                stage2Left.setPwmDisable();
            } else {
                stage1Left.setPwmEnable();
                stage2Left.setPwmEnable();

                stage1Left.setPosition(leftstage1pos);
                stage2Left.setPosition(leftstage2pos);
            }

            if (enableright == 0) {
                stage1Right.setPwmDisable();
                stage2Right.setPwmDisable();
            } else {
                stage1Right.setPwmEnable();
                stage2Right.setPwmEnable();

                stage1Right.setPosition(rightstage1pos);
                stage2Right.setPosition(rightstage2pos);
            }

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

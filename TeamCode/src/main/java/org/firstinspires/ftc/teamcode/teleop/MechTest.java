package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;

@TeleOp(name = "MechTest", group = "16481-Power-Play")
public class MechTest extends LinearOpMode {


    DcMotor RFMotor;
    DcMotor RBMotor;
    DcMotor LFMotor;
    DcMotor LBMotor;



    @Override
    public void runOpMode() throws InterruptedException {

        RFMotor = hardwareMap.get(DcMotor.class,"Fr");
        RBMotor = hardwareMap.get(DcMotor.class,"Br");
        LFMotor = hardwareMap.get(DcMotor.class,"Fl");
        LBMotor = hardwareMap.get(DcMotor.class,"Bl");


//        public void mecMove(){
//
//            double vertical;
//            double horizontal;
//            double pivot;
//            vertical=-gamepad1.left_stick_y;
//            horizontal=gamepad1.left_stick_x;
//            pivot=gamepad1.right_stick_x;
//
//
//            RFMotor. setPower (pivot + (-vertical + horizontal));
//            RBMotor. setPower (pivot + (-vertical - horizontal));
//            LFMotor. setPower(pivot + (-vertical - horizontal));
//            RFMotor. setPower (pivot + (-vertical + horizontal));

//
//        }
//
//        while (opModeInInit()) {
//        }
//
//        while (!isStopRequested()) {
//
//        }
    }


}

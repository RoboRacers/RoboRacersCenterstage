package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotCore;


@TeleOp(name = "Rolling Intake Test", group = "16481-CenterStage")
//@Disabled /* COMMENT THIS OUT WHEN YOU CREATE A NEW OPMODE */
public class IntakeSlidesTest_LM3_BOT extends LinearOpMode {

    RobotCore robot;
    DcMotor rightSlides;
    DcMotor leftSlides;

    DcMotor intake;



    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotCore(hardwareMap);
        rightSlides = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlides = hardwareMap.get(DcMotor.class, "leftSlide");
        intake = hardwareMap.get(DcMotor.class, "intake");


        while (opModeInInit()) {
        }

        while (!isStopRequested()) {

            if (gamepad1.left_stick_y > 0.1) {
                rightSlides.setPower(-gamepad1.left_stick_y*.4);
            } else if (gamepad1.left_stick_y < -0.1) {
                rightSlides.setPower(-gamepad1.left_stick_y*.7);
            } else {
                rightSlides.setPower(0);
            }

            if (gamepad1.left_stick_y > 0.1) {
                leftSlides.setPower(gamepad1.left_stick_y*.4);
            } else if (gamepad1.left_stick_y < -0.1) {
                leftSlides.setPower(gamepad1.left_stick_y*.7);
            } else {
                leftSlides.setPower(0);
            }

            if (gamepad1.right_trigger != 0){
                intake.setPower(0.6);
            }

            // Telemetry
            telemetry.addData("Right Slide Motor", rightSlides.getCurrentPosition());
            telemetry.addData("Left Slide Motor", leftSlides.getCurrentPosition());
            telemetry.update();

            robot.update();
        }
    }
}

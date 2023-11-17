package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

@TeleOp(name = "Dual Slides Test", group = "Test")
public class DoubleSlidesTest extends LinearOpMode {

    DcMotor rightSlides;
    DcMotor leftSlides;

    @Override
    public void runOpMode() throws InterruptedException {

        rightSlides = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlides = hardwareMap.get(DcMotor.class, "leftSlides");



        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlides.setDirection(DcMotorSimple.Direction.REVERSE);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

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

            if (gamepad1.right_stick_y > 0.1) {
                leftSlides.setPower(-gamepad1.right_stick_y*.4);
            } else if (gamepad1.right_stick_y < -0.1) {
                leftSlides.setPower(-gamepad1.right_stick_y*.7);
            } else {
                leftSlides.setPower(0);
            }

            // Telemetry
            telemetry.addData("Right Slide Motor", rightSlides.getCurrentPosition());
            telemetry.addData("Left Slide Motor", leftSlides.getCurrentPosition());
            telemetry.update();

        }
    }
}

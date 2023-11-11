package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.List;

@TeleOp(name = "Slides Test", group = "Test")
public class SlidesTest extends LinearOpMode {

    DcMotor rightSlides;

    @Override
    public void runOpMode() throws InterruptedException {

        rightSlides = hardwareMap.get(DcMotor.class, "rightSlides");

        rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftStage1 = hardwareMap.get(ServoImplEx.class, "leftStage1");



        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }




        while (opModeInInit()) {
        }


        while (!isStopRequested()) {

            if (gamepad1.left_stick_y > 0.1) {
                rightSlides.setPower(-.3);
            } else if (gamepad1.left_stick_y < -0.1) {
                rightSlides.setPower(.7);
            } else {
                rightSlides.setPower(0);
            }

            // Telemetry
            telemetry.addData("Right Slide Motor", rightSlides.getCurrentPosition());
            //telemetry.addData("Left Servo Position", leftStage1.getPosition());
            telemetry.update();

        }
    }
}

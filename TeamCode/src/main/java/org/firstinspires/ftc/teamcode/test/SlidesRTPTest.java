package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.modules.statemachines.SlidesSM;
import org.firstinspires.ftc.teamcode.modules.subsystems.Slides;

@Config
@TeleOp(name = "Slides RTP Test", group = "Slides-Test")
public class SlidesRTPTest extends LinearOpMode {

    Slides slides;

    public static int setPoint = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        slides = new Slides(hardwareMap);

        slides.leftmotor.setTargetPosition(-setPoint);
        slides.rightmotor.setTargetPosition(setPoint);

        slides.leftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.rightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeInInit()) {
        }

        while (!isStopRequested()) {

            slides.leftmotor.setTargetPosition(setPoint);
            slides.rightmotor.setTargetPosition(setPoint);

            slides.rightmotor.setPower(0.7);
            slides.leftmotor.setPower(0.7);

            // Telemetry
            telemetry.addData("Setpoint", setPoint);
            telemetry.addData("Right Slide Motor", slides.rightmotor.getCurrentPosition());
            telemetry.addData("Left Slide Motor", slides.leftmotor.getCurrentPosition());
            telemetry.addData("Right Target", slides.rightmotor.getTargetPosition());
            telemetry.addData("Left Target", slides.leftmotor.getTargetPosition());
            telemetry.addData("Right Power", slides.rightmotor.getPower());
            telemetry.addData("Left Power", slides.rightmotor.getPower());
            telemetry.update();

        }
    }
}

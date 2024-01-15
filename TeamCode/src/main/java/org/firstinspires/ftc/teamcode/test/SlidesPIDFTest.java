package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.modules.subsystems.Slides;

import java.util.List;

@Config
@TeleOp(name = "Slides PID Test", group = "Slides-Test")
public class SlidesPIDFTest extends LinearOpMode {

    Slides slides;

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;

    public static int setPoint = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        slides = new Slides(hardwareMap);

        while (opModeInInit()) {
        }

        while (!isStopRequested()) {

            slides.setPID(kP,kI,kD);
            slides.setTargetPosition(setPoint);

            // Telemetry
            telemetry.addData("Left Slide Motor", slides.leftmotor.getCurrentPosition());
            telemetry.update();

        }
    }
}

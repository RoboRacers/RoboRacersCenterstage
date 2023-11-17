package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import java.util.List;

@Config
@TeleOp(name = "Slides PIDF Test", group = "Test")
public class SlidesPIDFTest extends LinearOpMode {

    DcMotorImplEx rightSlides;
    DcMotorImplEx leftSlides;

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;

    public static int setPoint = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        rightSlides = hardwareMap.get(DcMotorImplEx.class, "rightSlide");
        leftSlides = hardwareMap.get(DcMotorImplEx.class, "leftSlides");



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

            rightSlides.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                    new PIDCoefficients(kP,kI,kD));

            leftSlides.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                    new PIDCoefficients(kP,kI,kD));

            rightSlides.setTargetPosition(setPoint);
            leftSlides.setTargetPosition(setPoint);
            
            // Telemetry
            telemetry.addData("Right Slide Motor", rightSlides.getCurrentPosition());
            telemetry.addData("Left Slide Motor", leftSlides.getCurrentPosition());
            telemetry.update();

        }
    }
}

package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import java.util.List;
@Deprecated
@Disabled
@TeleOp(name = "Slides RTP Test", group = "Slides-Test")
public class SlidesRunToPositionTest extends LinearOpMode {


    DcMotor rightSlides;
    DcMotor leftSlides;

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;

    public static int setPoint = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        rightSlides = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlides = hardwareMap.get(DcMotor.class, "leftSlide");



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

            rightSlides.setTargetPosition(setPoint);
            leftSlides.setTargetPosition(setPoint);

            // Telemetry
            telemetry.addData("Right Slide Motor", rightSlides.getCurrentPosition());
            telemetry.addData("Left Slide Motor", leftSlides.getCurrentPosition());
            telemetry.update();

        }
    }
}

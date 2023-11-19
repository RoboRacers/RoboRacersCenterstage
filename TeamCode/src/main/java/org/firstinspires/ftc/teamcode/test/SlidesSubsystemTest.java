package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.modules.statemachines.SlidesSM;
import org.firstinspires.ftc.teamcode.modules.subsystems.Slides;

import java.util.List;

@Config
@TeleOp(name = "Slides Subsystem Test", group = "Test")
public class SlidesSubsystemTest extends LinearOpMode {

    Slides slides;

    public static int setPoint = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        slides = new Slides(hardwareMap);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        while (opModeInInit()) {
        }

        while (!isStopRequested()) {

            slides.setTargetPosition(setPoint);
            slides.setPower(0.3);

            // Telemetry
            telemetry.addData("Right Slide Motor", slides.rightmotor.getCurrentPosition());
            telemetry.addData("Left Slide Motor", slides.leftmotor.getCurrentPosition());
            telemetry.update();

        }
    }
}

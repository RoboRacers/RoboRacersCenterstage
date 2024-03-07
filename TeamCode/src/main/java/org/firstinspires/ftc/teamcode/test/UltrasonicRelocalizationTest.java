package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.modules.drive.MecanumDrive;

import java.util.List;


@TeleOp(name = "Ultrasonic Relocalization Test", group = "Test")
public class UltrasonicRelocalizationTest extends LinearOpMode {

    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drive.setPoseEstimate(new Pose2d(-43, 62.00, Math.toRadians(-90)));
        while(opModeInInit());

        while (!isStopRequested()) {
            drive.update();

            double relocalizationY = 72 - (drive.leftUltrasonic.getDistance(DistanceUnit.INCH) + 9.00);

            if (gamepad1.dpad_up) {
                drive.setPoseEstimate(new Pose2d(0,0,0));
            }

            // Telemetry
            telemetry.addLine("Ultrasonic Testing");
            telemetry.addData("Left Ultrasonic", drive.leftUltrasonic.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Ultrasonic", drive.rightUltrasonic.getDistance(DistanceUnit.INCH));
            telemetry.addData("Drive pose", drive.getPoseEstimate());
            telemetry.addData("Relocalized Y", relocalizationY);
            telemetry.update();

        }
    }
}

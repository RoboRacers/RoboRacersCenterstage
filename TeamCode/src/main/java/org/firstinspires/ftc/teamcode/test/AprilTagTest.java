package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.modules.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.modules.subsystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "April Tag Test", group = "Test")
public class AprilTagTest extends LinearOpMode {

    public Vision vision;
    public MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        vision = new Vision(hardwareMap);
        drive = new MecanumDrive(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {

            Vector2d aprilTagPose = new Vector2d(0,0);

            if (!vision.getDetections().isEmpty()) {
                AprilTagDetection detection = vision.getDetections().get(0);
                aprilTagPose = vision.getFCPosition(detection, drive.getPoseEstimate().getHeading());
            }

            drive.update();

            telemetry.addData("April Tag Pose", aprilTagPose);
            telemetry.addData("Drive Pose", drive.getPoseEstimate());
            telemetry.update();

        }
    }
}



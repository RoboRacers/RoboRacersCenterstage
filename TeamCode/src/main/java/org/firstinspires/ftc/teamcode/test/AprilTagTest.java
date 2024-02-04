package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.modules.subsystems.Vision;
import org.firstinspires.ftc.teamcode.modules.util.RoadrunnerUtil.DashboardUtil;
import org.firstinspires.ftc.teamcode.modules.vision.Transform3d;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.ArrayList;

@TeleOp(name = "April Tag Detection Test", group = "Test")
public class AprilTagTest extends LinearOpMode {

    public Vision vision;

    @Override
    public void runOpMode() throws InterruptedException {

        vision = new Vision(hardwareMap);

        waitForStart();


        while (!isStopRequested()) {

            if (gamepad1.dpad_down) {
                vision.visionPortal.stopStreaming();

            } else if (gamepad1.dpad_up) {
                vision.visionPortal.resumeStreaming();
            }

            telemetry.addData("Portal State", vision.visionPortal.getCameraState());
            telemetry.addData("Detections", vision.tagProcessor.getDetections());

            ArrayList<Pose2d> poses = vision.getAprilTagPoses();
            TelemetryPacket packet = new TelemetryPacket();
            if (poses.isEmpty()) {
                poses.add(
                        new Pose2d(0,0,0)
                );
            }
            DashboardUtil.drawRobot(packet.fieldOverlay(), poses.get(0));

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.addData("Possible Pose", poses);
            telemetry.update();

        }
        vision.visionPortal.close();

    }

}



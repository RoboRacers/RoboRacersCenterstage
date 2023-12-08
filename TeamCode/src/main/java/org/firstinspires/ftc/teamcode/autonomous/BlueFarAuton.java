package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.subsystems.PropDetection;
import org.firstinspires.ftc.teamcode.util.SpikeMarkerLocation;
import org.firstinspires.ftc.teamcode.modules.statemachines.IntakeSM;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

// Localization is doesn't show drift, follower if it does

@Config
@Autonomous(name = "Blue Far Side Auton", group = "16481-Centerstage")
public class BlueFarAuton extends LinearOpMode{

    RobotCore robot;

    SpikeMarkerLocation spikeMarkerLocation = SpikeMarkerLocation.CENTER; // Defaults to center

    @Override
    public void runOpMode() {

        robot = new RobotCore(hardwareMap);

        Pose2d startLocation = new Pose2d(-40, 62.00, Math.toRadians(90));

        TrajectorySequence CenterNoCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .setReversed(true)
                .splineTo(new Vector2d(-36.32, 28.93), Math.toRadians(270.00))
                .setReversed(false)
                .splineTo(new Vector2d(-57.03, 35.77), Math.toRadians(266.35))
                .setReversed(true)
                .splineTo(new Vector2d(-43.16, -0.28), Math.toRadians(4.76))
                .splineTo(new Vector2d(16.73, 3.42), Math.toRadians(17.18))
                .splineTo(new Vector2d(47.23, 34.66), Math.toRadians(-11.31))
                .setReversed(false)
                .splineTo(new Vector2d(42.24, 9.70), Math.toRadians(-60.34))
                .splineTo(new Vector2d(61.46, 8.04), Math.toRadians(2.39))
                .build();


        TrajectorySequence LeftNoCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .setReversed(true)
                .splineTo(new Vector2d(-28.30, 33.07), Math.toRadians(-79.08))
                .setReversed(false)
                .splineTo(new Vector2d(-51.94, 34.66), Math.toRadians(176.15))
                .splineTo(new Vector2d(-55.12, 13.41), Math.toRadians(-8.95))
                .setReversed(true)
                .splineTo(new Vector2d(5.46, 12.41), Math.toRadians(11.51))
                .splineTo(new Vector2d(46.97, 35.85), Math.toRadians(-10.09))
                .setReversed(false)
                .splineTo(new Vector2d(36.45, 20.16), Math.toRadians(222.11))
                .setReversed(true)
                .splineTo(new Vector2d(65.25, 12.41), Math.toRadians(-32.81))
                .build();

        TrajectorySequence RightNoCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .setReversed(true)
                .splineTo(new Vector2d(-43.53, 30.22), Math.toRadians(186.44))
                .setReversed(false)
                .splineTo(new Vector2d(-35.03, 16.91), Math.toRadians(-61.39))
                .setReversed(true)
                .splineTo(new Vector2d(-35.03, 1.02), Math.toRadians(-73.57))
                .splineTo(new Vector2d(15.44, 6.56), Math.toRadians(27.88))
                .splineTo(new Vector2d(46.68, 34.66), Math.toRadians(0.00))
                .setReversed(false)
                .splineTo(new Vector2d(43.16, 9.52), Math.toRadians(-52.43))
                .splineTo(new Vector2d(62.94, 8.78), Math.toRadians(18.43))
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap
                .get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        PropDetection teamPropDetectionPipeline = new PropDetection(camera, telemetry);

        while(!isStopRequested() && !opModeIsActive()) {
            // Vision code here
            if (gamepad1.left_bumper) {
                spikeMarkerLocation = teamPropDetectionPipeline.getDirection();
            }

            if (gamepad1.square) {
                spikeMarkerLocation = SpikeMarkerLocation.LEFT;
            } else if (gamepad1.circle) {
                spikeMarkerLocation = SpikeMarkerLocation.CENTER;
            } else if (gamepad1.triangle) {
                spikeMarkerLocation = SpikeMarkerLocation.RIGHT;
            }

            telemetry.addData("Spike Marker Location", spikeMarkerLocation);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        robot.drive.setPoseEstimate(startLocation);

        // Runs the trajectory based on the start location
        switch (spikeMarkerLocation) {
            case LEFT:
                robot.drive.followTrajectorySequence(LeftNoCycle);
                break;
            case CENTER:
                robot.drive.followTrajectorySequence(CenterNoCycle);
                break;
            case RIGHT:
                robot.drive.followTrajectorySequence(RightNoCycle);
                break;
        }

    }

}

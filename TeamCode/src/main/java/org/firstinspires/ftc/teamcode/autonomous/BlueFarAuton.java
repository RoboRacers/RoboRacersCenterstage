package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.drive.ThreeTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.modules.util.SpikeMarkerLocation;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;

import java.util.List;

// Localization is doesn't show drift, follower if it does

@Config
@Autonomous(name = "Blue Far Side Auton", group = "16481-Centerstage")
public class BlueFarAuton extends LinearOpMode{

    RobotCore robot;

    SpikeMarkerLocation spikeMarkerLocation = SpikeMarkerLocation.CENTER; // Defaults to center

    @Override
    public void runOpMode() {

        robot = new RobotCore(hardwareMap);

        Pose2d startLocation = new Pose2d(-40,  62.00, Math.toRadians(-90));

        TrajectorySequence LeftNoCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .setReversed(true)
                .splineTo(new Vector2d(-31.05, 34.25), Math.toRadians(-22.44))
                .setReversed(false)
                .splineTo(new Vector2d(-46.52, 49.73), Math.toRadians(-270.00))
                .setReversed(true)
                .splineTo(new Vector2d(-45.39, 14.82), Math.toRadians(-39.67))
                .splineTo(new Vector2d(-12.55, 8.59), Math.toRadians(4.55))
                .splineTo(new Vector2d(24.25, 10.29), Math.toRadians(7.52))
                .splineTo(new Vector2d(35.76, 20.48), Math.toRadians(40.24))
                // Scoring
                .addDisplacementMarker(() -> {
                    robot.slides.setTargetPosition(800);
                    robot.slides.setPower(.8);
                })
                .splineTo(new Vector2d(46.00, 41.00), Math.toRadians(0.00))
                .addDisplacementMarker(() -> {
                })
                .waitSeconds(.75)
                .setReversed(false)
                .splineTo(new Vector2d(40.54,41.00), Math.toRadians(180.00))
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    robot.slides.setTargetPosition(0);
                })
                .splineTo(new Vector2d(42.24, 9.70), Math.toRadians(-60.34))
                .splineTo(new Vector2d(61.46, 9.50), Math.toRadians(2.39))
                .build();

        TrajectorySequence CenterNoCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .addDisplacementMarker(() -> {
                    robot.intake.engageLock(true,true);
                    robot.intake.flipDeposit();
                })
                .splineToLinearHeading(new Pose2d(-33.12, 18.31, Math.toRadians(-36.10)), Math.toRadians(-36.10))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    robot.intake.setIntakePower(-0.3);
                })
                .waitSeconds(1.333)
                .setReversed(true)
                .splineTo(new Vector2d(-60.15, 11.61), Math.toRadians(177.51))
                //.splineTo(new Vector2d(-15.57, 10.47), Math.toRadians(2.06))
                //.splineTo(new Vector2d(28.78, 16.51), Math.toRadians(17.74))
                .build();

        TrajectorySequence RightNoCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .setReversed(true)
                .splineTo(new Vector2d(-42.30, 34.49), Math.toRadians(-135.00))
                .setReversed(false)
                .splineTo(new Vector2d(-36.07, 53.24), Math.toRadians(-262.41))
                .setReversed(true)
                .splineTo(new Vector2d(-35.00, 15.34), Math.toRadians(-45.00))
                .splineTo(new Vector2d(9.28, 5.57), Math.toRadians(5.78))
                .splineTo(new Vector2d(38.98, 21.98), Math.toRadians(43.26))
                .addSpatialMarker(new Vector2d(33,-33),() -> {
                    robot.slides.setTargetPosition(800);
                    robot.slides.setPower(.8);
                })
                // To backboard
                .splineTo(new Vector2d(49.50, 27.3), Math.toRadians(0.00))
                .addDisplacementMarker(() -> {
                })
                .waitSeconds(.75)
                .setReversed(false)
                .splineTo(new Vector2d(39, 27.3), Math.toRadians(180))
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    robot.slides.setTargetPosition(0);
                })
                .splineTo(new Vector2d(42.24, 9.70), Math.toRadians(-60.34))
                .splineTo(new Vector2d(61.46, 9.50), Math.toRadians(2.39))
                .build();

        // Close claw

        //robot.vision.startPropDetection();

        boolean manualPropControl = false;

        while(!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("SELF CHECK -----");

            // Checks if the positions of the encoders to make sure they are not unplugged
            robot.drive.updatePoseEstimate();
            ThreeTrackingWheelLocalizer localizer = (ThreeTrackingWheelLocalizer) robot.drive.getLocalizer();
            List<Double> deadwheelPositions = localizer.getWheelPositions();

            telemetry.addData("Left Encoder Pos", deadwheelPositions.get(0));
            telemetry.addData("Right Encoder Pos", deadwheelPositions.get(1));
            telemetry.addData("Perpendicular Encoder Pos", deadwheelPositions.get(2));

            if (deadwheelPositions.get(0) == 0) {
                telemetry.addLine("LEFT ENCODER UNPLUGGED, Check wiring of Port x");
            }
            if (deadwheelPositions.get(1) == 0) {
                telemetry.addLine("RIGHT ENCODER UNPLUGGED, Check wiring of Port x");
            }
            if (deadwheelPositions.get(2) == 0) {
                telemetry.addLine("PERPENDICULAR ENCODER UNPLUGGED, Check wiring of Port x");
            }


            // Vision code here
            telemetry.addLine("VISION -----");

            // Switch between manual and automatic vision control
            manualPropControl = true;
            if (gamepad1.left_bumper) {
                manualPropControl = true;
            } else if (gamepad1.right_bumper) {
                //manualPropControl = false;
            }

            if (!manualPropControl) {
                telemetry.addLine("Prop Detection mode is AUTOMATIC");
                if (robot.vision.getDirection() != null) {
                    spikeMarkerLocation = robot.vision.getDirection();
                    telemetry.addData("Spike Marker Location", spikeMarkerLocation);
                } else {
                    telemetry.addLine("Camera not initialized");
                }
            } else {
                telemetry.addLine("Prop Detection is MANUAL");
                if (gamepad1.square) {
                    spikeMarkerLocation = SpikeMarkerLocation.LEFT;
                } else if (gamepad1.circle) {
                    spikeMarkerLocation = SpikeMarkerLocation.CENTER;
                } else if (gamepad1.triangle) {
                    spikeMarkerLocation = SpikeMarkerLocation.RIGHT;
                }
                telemetry.addData("Spike Marker Location", spikeMarkerLocation);
            }

            telemetry.update();
        }

        //robot.vision.stopPropDetection();

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

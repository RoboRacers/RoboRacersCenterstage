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
@Autonomous(name = "Red Far Side Auton", group = "16481-Centerstage")
public class RedFarAuton extends LinearOpMode{

    RobotCore robot;

    SpikeMarkerLocation spikeMarkerLocation = SpikeMarkerLocation.CENTER; // Defaults to center

    @Override
    public void runOpMode() {

        robot = new RobotCore(hardwareMap);

        Pose2d startLocation = new Pose2d(-40, -62.00, Math.toRadians(-90));

        TrajectorySequence CenterNoCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .setReversed(true)
                // Push purple pixel
                .splineTo(new Vector2d(-36.32, -29.93), Math.toRadians(-270.00))
                .setReversed(false)
                .splineTo(new Vector2d(-48.03, -47.84), Math.toRadians(220.60))
                .setReversed(true)
                .splineTo(new Vector2d(-55.96, -15.38), Math.toRadians(66.80))
                .splineTo(new Vector2d(-2.36, -2.36), Math.toRadians(2.79))
                .splineTo(new Vector2d(33.88, -17.08), Math.toRadians(-35.22))
                // Scoring
                .addDisplacementMarker(() -> {
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.EXTEND_WITH_PIXEL
                    );
                    robot.slides.setTargetPosition(820);
                    robot.slides.setPower(.8);
                })
                // To backboard
                .splineTo(new Vector2d(48.50, -34.90), Math.toRadians(0.00))
                .addDisplacementMarker(() -> {
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.OPEN_CLAW
                    );
                })
                .waitSeconds(.75)
                .setReversed(false)
                .splineTo(new Vector2d(39, -34.90), Math.toRadians(-180))
                .addDisplacementMarker(() -> {
                    robot.slides.setTargetPosition(0);
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.CLOSED_WITH_PIXEL
                    );
                })
                .setReversed(true)
                .splineTo(new Vector2d(42.24, -9.70), Math.toRadians(60.34))
                .splineTo(new Vector2d(61.46, -9.50), Math.toRadians(-2.39))
                .build();


        TrajectorySequence LeftNoCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .setReversed(true)
                .splineTo(new Vector2d(-31.05, -34.25), Math.toRadians(22.44))
                .setReversed(false)
                .splineTo(new Vector2d(-46.52, -49.73), Math.toRadians(270.00))
                .setReversed(true)
                .splineTo(new Vector2d(-45.39, -14.82), Math.toRadians(39.67))
                .splineTo(new Vector2d(-12.55, -8.59), Math.toRadians(-4.55))
                .splineTo(new Vector2d(24.25, -10.29), Math.toRadians(-7.52))
                .splineTo(new Vector2d(35.76, -20.48), Math.toRadians(-40.24))
                // Scoring
                .addDisplacementMarker(() -> {
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.EXTEND_WITH_PIXEL
                    );
                    robot.slides.setTargetPosition(800);
                    robot.slides.setPower(.8);
                })
                .splineTo(new Vector2d(48.00, -41.00), Math.toRadians(0.00))
                .addDisplacementMarker(() -> {
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.OPEN_CLAW
                    );
                })
                .waitSeconds(.75)
                .setReversed(false)
                .splineTo(new Vector2d(40.54,-41.00), Math.toRadians(-180.00))
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    robot.slides.setTargetPosition(0);
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.CLOSED_WITH_PIXEL
                    );
                })
                .splineTo(new Vector2d(42.24, -9.70), Math.toRadians(60.34))
                .splineTo(new Vector2d(61.46, -9.50), Math.toRadians(-2.39))
                .build();

        TrajectorySequence RightNoCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .setReversed(true)
                .splineTo(new Vector2d(-42.30, -34.49), Math.toRadians(135.00))
                .setReversed(false)
                .splineTo(new Vector2d(-36.07, -53.24), Math.toRadians(262.41))
                .setReversed(true)
                .splineTo(new Vector2d(-35.00, -15.34), Math.toRadians(45.00))
                .splineTo(new Vector2d(9.28, -5.57), Math.toRadians(-5.78))
                .splineTo(new Vector2d(38.98, -21.98), Math.toRadians(-43.26))
                .addSpatialMarker(new Vector2d(33,-33),() -> {
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.EXTEND_WITH_PIXEL
                    );
                    robot.slides.setTargetPosition(800);
                    robot.slides.setPower(.8);
                })
                // To backboard
                .splineTo(new Vector2d(49.50, -27.3), Math.toRadians(0.00))
                .addDisplacementMarker(() -> {
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.OPEN_CLAW
                    );
                })
                .waitSeconds(.75)
                .setReversed(false)
                .splineTo(new Vector2d(39, -27.3), Math.toRadians(-180))
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    robot.slides.setTargetPosition(0);
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.CLOSED_WITH_PIXEL
                    );
                })
                .splineTo(new Vector2d(42.24, -9.70), Math.toRadians(60.34))
                .splineTo(new Vector2d(61.46, -9.50), Math.toRadians(-2.39))
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap
                .get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        PropDetection teamPropDetectionPipeline = new PropDetection(camera, telemetry);

        robot.intake.claw.setPosition(0.4);

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

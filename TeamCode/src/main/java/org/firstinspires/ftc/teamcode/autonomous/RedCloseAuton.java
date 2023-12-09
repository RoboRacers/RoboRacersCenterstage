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
@Autonomous(name = "Red Close Side Auton", group = "16481-Centerstage")
public class RedCloseAuton extends LinearOpMode{

    RobotCore robot;

    SpikeMarkerLocation spikeMarkerLocation = SpikeMarkerLocation.CENTER; // Defaults to center

    @Override
    public void runOpMode() {

        robot = new RobotCore(hardwareMap);

        Pose2d startLocation = new Pose2d(15.85, -62.00, Math.toRadians(-90));



        TrajectorySequence LeftNoCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .setReversed(true)
                .splineTo(new Vector2d(11.42, -34.80), Math.toRadians(-270.00))
                .splineTo(new Vector2d(17.50, -34.38), Math.toRadians(18.43))
                .setReversed(false)
                .splineTo(new Vector2d(10.89, -31.49), Math.toRadians(75.96))
                .setReversed(true)
                .splineTo(new Vector2d(17.86, -44.56), Math.toRadians(1.64))
                .addDisplacementMarker(() -> {
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.EXTEND_WITH_PIXEL
                    );
                    robot.slides.setTargetPosition(800);
                    robot.slides.setPower(.8);
                })
                // To backboard
                .splineTo(new Vector2d(45.11, -42.36), Math.toRadians(0.00))
                .addDisplacementMarker(() -> {
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.OPEN_CLAW
                    );
                })
                .waitSeconds(.75)
                .setReversed(false)
                .splineTo(new Vector2d(40.54,-35.36), Math.toRadians(-180.00))
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    robot.slides.setTargetPosition(0);
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.CLOSED_WITH_PIXEL
                    );
                })
                .splineTo(new Vector2d(58.01, -60.21), Math.toRadians(0.00))
                .build();

        //DONNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNEEEEE
        TrajectorySequence CenterNoCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .setReversed(true)
                .splineTo(new Vector2d(12.95, -28.00), Math.toRadians(-266.31))
                .setReversed(false)
                .splineTo(new Vector2d(7.46, -40.02), Math.toRadians(-143.13))
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.EXTEND_WITH_PIXEL
                    );
                    robot.slides.setTargetPosition(820);
                    robot.slides.setPower(.8);
                })
                // To backboard
                .splineTo(new Vector2d(48.50, -35.90), Math.toRadians(0.00))
                .addDisplacementMarker(() -> {
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.OPEN_CLAW
                    );
                })
                .waitSeconds(.75)
                .setReversed(false)
                .splineTo(new Vector2d(39, -35.90), Math.toRadians(-180))
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    robot.slides.setTargetPosition(0);
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.CLOSED_WITH_PIXEL
                    );
                })
                .splineTo(new Vector2d(58,-60), Math.toRadians(0))
                .build();


        TrajectorySequence RightNoCycle = robot.drive.trajectorySequenceBuilder(new Pose2d(15.85, 62.00, Math.toRadians(90)))
                .setReversed(true)
                // Pushing purple pixel
                .splineTo(new Vector2d(11.56, -44.15), Math.toRadians(-270.00))
                .splineTo(new Vector2d(5.27, -32.90), Math.toRadians(-232.84))
                .setReversed(false)
                // Moving backwards
                .splineTo(new Vector2d(14.32, -44.35), Math.toRadians(-45.92))
                .setReversed(true)
                .addSpatialMarker(new Vector2d(33,-33),() -> {
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.EXTEND_WITH_PIXEL
                    );
                    robot.slides.setTargetPosition(800);
                    robot.slides.setPower(.8);
                })
                // To backboard
                .splineTo(new Vector2d(49.50, -28.3), Math.toRadians(0.00))
                .addDisplacementMarker(() -> {
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.OPEN_CLAW
                    );
                })
                .waitSeconds(.75)
                .setReversed(false)
                .splineTo(new Vector2d(39, -28.3), Math.toRadians(-180))
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    robot.slides.setTargetPosition(0);
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.CLOSED_WITH_PIXEL
                    );
                })
                .splineTo(new Vector2d(58,-60), Math.toRadians(0))
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap
                .get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        PropDetection teamPropDetectionPipeline = new PropDetection(camera, telemetry);

        while(!isStopRequested() && !opModeIsActive()) {
            // Vision code here
            if (gamepad1.left_bumper) { // Remove this for actual operation
                try {
                    // Catches any null pointer exceptions from the camera not being initialized
                    if (teamPropDetectionPipeline != null) {
                        spikeMarkerLocation = teamPropDetectionPipeline.getDirection();
                    } else {
                        telemetry.addLine("Camera not initialized");
                        telemetry.update();
                    }
                } catch (Exception e) {
                    telemetry.addLine("Exception! " + e);
                    telemetry.update();
                }

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
        robot.intake.claw.setPosition(0.05);

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

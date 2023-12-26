package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.modules.statemachines.SlidesSM;
import org.firstinspires.ftc.teamcode.modules.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.SpikeMarkerLocation;
import org.firstinspires.ftc.teamcode.modules.statemachines.IntakeSM;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.List;
import java.util.function.Consumer;

// Localization is doesn't show drift, follower if it does

@Config
@Autonomous(name = "Blue Close Side Auton", group = "16481-Centerstage")
public class BlueCloseAuton extends LinearOpMode{

    RobotCore robot;

    SpikeMarkerLocation spikeMarkerLocation = SpikeMarkerLocation.CENTER; // Defaults to center

    @Override
    public void runOpMode() {

        robot = new RobotCore(hardwareMap);

        Pose2d startLocation = new Pose2d(15.85, 62.00, Math.toRadians(90));

        TrajectorySequence LeftNoCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .setReversed(true)
                .splineTo(new Vector2d(18.00, 33.70), Math.toRadians(-25.20))
                .setReversed(false)
                .splineTo(new Vector2d(6.00, 43.00), Math.toRadians(142.70))
                .setReversed(true)
                .splineTo(new Vector2d(26.89, 48.60), Math.toRadians(-19.54))
                .addDisplacementMarker(() -> {
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.EXTEND_WITH_PIXEL
                    );
                    robot.slides.setTargetPosition(800);
                    robot.slides.setPower(.8);
                })
                .splineTo(new Vector2d(47.00, 43.5), Math.toRadians(0.00))
                .addDisplacementMarker(() -> {
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.OPEN_CLAW
                    );
                })
                .waitSeconds(.75)
                .setReversed(false)
                .splineTo(new Vector2d(40.54,43.5), Math.toRadians(180.00))
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    robot.slides.setTargetPosition(0);
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.CLOSED_WITH_PIXEL
                    );
                })
                .splineTo(new Vector2d(58.01, 60.21), Math.toRadians(0.00))
                .build();

        TrajectorySequence CenterNoCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .setReversed(true)
                .splineTo(new Vector2d(12.95, 29.70), Math.toRadians(266.31))
                .setReversed(false)
                .splineTo(new Vector2d(7.46, 40.02), Math.toRadians(143.13))
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.EXTEND_WITH_PIXEL
                    );
                    robot.slides.setTargetPosition(820);
                    robot.slides.setPower(.8);
                })
                // To backboard
                .splineTo(new Vector2d(47.50, 35.90), Math.toRadians(0.00))
                .addDisplacementMarker(() -> {
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.OPEN_CLAW
                    );
                })
                .waitSeconds(.75)
                .setReversed(false)
                .splineTo(new Vector2d(39, 35.90), Math.toRadians(180))
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    robot.slides.setTargetPosition(0);
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.CLOSED_WITH_PIXEL
                    );
                })
                .splineTo(new Vector2d(58,60), Math.toRadians(0))
                .build();

        TrajectorySequence RightNoCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .setReversed(true)
                // Pushing purple pixel
                .splineTo(new Vector2d(11.56, 46.20), Math.toRadians(270.00))
                .splineTo(new Vector2d(5.27, 32.90), Math.toRadians(232.84))
                .setReversed(false)
                // Moving backwards
                .splineTo(new Vector2d(14.32, 44.35), Math.toRadians(45.92))
                .setReversed(true)
                .addSpatialMarker(new Vector2d(33,33),() -> {
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.EXTEND_WITH_PIXEL
                    );
                    robot.slides.setTargetPosition(800);
                    robot.slides.setPower(.8);
                })
                // To backboard
                .splineTo(new Vector2d(47.00, 29.3), Math.toRadians(0.00))
                .addDisplacementMarker(() -> {
                    /*
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.OPEN_CLAW
                    );

                     */
                })
                .waitSeconds(.75)
                .setReversed(false)
                .splineTo(new Vector2d(39, 28.3), Math.toRadians(180))
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    robot.slides.setTargetPosition(0);
                    robot.intake.statemachine.transition(
                            IntakeSM.EVENT.CLOSED_WITH_PIXEL
                    );
                })
                .splineTo(new Vector2d(58,60), Math.toRadians(0))
                .build();

        // Close claw
        robot.intake.statemachine.transition(IntakeSM.EVENT.CLOSE_CLAW);

        robot.vision.startPropDetection();

        boolean manualPropControl = false;

        while(!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("SELF CHECK -----");

            // Checks if the positions of the encoders to make sure they are not unplugged
            robot.drive.updatePoseEstimate();
            StandardTrackingWheelLocalizer localizer = (StandardTrackingWheelLocalizer) robot.drive.getLocalizer();
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
            if (gamepad1.left_bumper) {
                manualPropControl = true;
            } else if (gamepad1.right_bumper) {
                manualPropControl = false;
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

        robot.vision.stopPropDetection();

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

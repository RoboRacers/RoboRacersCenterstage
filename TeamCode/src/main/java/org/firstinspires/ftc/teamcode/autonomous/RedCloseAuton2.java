package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.drive.ThreeTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.modules.statemachines.SlidesSM;
import org.firstinspires.ftc.teamcode.modules.subsystems.Vision;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.modules.util.SpikeMarkerLocation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

// Localization is doesn't show drift, follower if it does

@Config
@Autonomous(name = "Red Close Side Auton", group = "16481-Centerstage")
public class RedCloseAuton2 extends LinearOpMode{

    RobotCore robot;

    Vision.TeamPropPipeline teamPropDetectionPipeline = null;

    SpikeMarkerLocation spikeMarkerLocation = SpikeMarkerLocation.CENTER; // Defaults to center

    @Override
    public void runOpMode() {

        OpenCvCamera camera;

        robot = new RobotCore(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap
                .get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                teamPropDetectionPipeline = new Vision.TeamPropPipeline();
                camera.setPipeline(teamPropDetectionPipeline);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        Pose2d startLocation = new Pose2d(15.85, -62.00, Math.toRadians(90));

        TrajectorySequence LeftNoCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .addDisplacementMarker(() -> {
                    robot.intake.engageLock(true,true);
                    robot.intake.flipDeposit();
                })
                .splineToConstantHeading(new Vector2d(22.9, -40.00), Math.toRadians(90))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(27.30, -50.00), Math.toRadians(90))
                // Go to backboard
                .splineTo(new Vector2d(49.25, -40.25), Math.toRadians(0.00))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0);
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_RTP
                    );

                    robot.slides.setTargetPosition(-700);
                    robot.slides.setPower(0.8);

                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.clearHigherLock();
                    robot.intake.clearLowerLock();
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    robot.intake.flipIntake();
                    robot.slides.setTargetPosition(0);
                    robot.slides.setPower(0.8);

                })
                .splineToConstantHeading(new Vector2d(40.00, -26.00), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(46.50, -43.0), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(53.43, -55.83), Math.toRadians(0.00))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Unpower slides
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_MANUAL
                    );
                    robot.slides.setPower(0);
                })
                .build();

        TrajectorySequence CenterNoCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .addDisplacementMarker(() -> {
                    robot.intake.engageLock(true,true);
                    robot.intake.flipDeposit();
                })
                .splineToConstantHeading(new Vector2d(14.00, -31.55), Math.toRadians(90))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(14.00, -38), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(14.00, -41.00, Math.toRadians(45)), Math.toRadians(45))
                // Go to backboard
                .splineTo(new Vector2d(49.25, -34.50), Math.toRadians(0.00))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0);
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_RTP
                    );

                    robot.slides.setTargetPosition(-700);
                    robot.slides.setPower(0.8);

                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.clearHigherLock();
                    robot.intake.clearLowerLock();
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    robot.intake.flipIntake();
                    robot.slides.setTargetPosition(0);
                    robot.slides.setPower(0.8);

                })
                .splineToConstantHeading(new Vector2d(40.00, -26.00), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(46.50, -43.0), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(53.43, -58.83), Math.toRadians(0.00))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Unpower slides
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_MANUAL
                    );
                    robot.slides.setPower(0);
                })
                .build();


        TrajectorySequence RightNoCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .addDisplacementMarker(() -> {
                    robot.intake.engageLock(true,true);
                    robot.intake.flipDeposit();
                })
                .splineTo(new Vector2d(8, -39), Math.toRadians(135))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(18, -39), Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(30, -34, Math.toRadians(0)), Math.toRadians(0))
                // Go to backboard
                .splineTo(new Vector2d(49.25, -26.00), Math.toRadians(0.00))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0);
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_RTP
                    );

                    robot.slides.setTargetPosition(-700);
                    robot.slides.setPower(0.8);

                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.clearHigherLock();
                    robot.intake.clearLowerLock();
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    robot.intake.flipIntake();
                    robot.slides.setTargetPosition(0);
                    robot.slides.setPower(0.8);

                })
                .splineToConstantHeading(new Vector2d(40.00, -26.00), Math.toRadians(0.00))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(53.43, -58.83), Math.toRadians(0.00))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Unpower slides
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_MANUAL
                    );
                    robot.slides.setPower(0);
                })
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
            if (gamepad1.left_bumper) {
                manualPropControl = true;
            } else if (gamepad1.right_bumper) {
                manualPropControl = false;
            }

            if (!manualPropControl) {
                telemetry.addLine("Prop Detection mode is AUTOMATIC");
                if (teamPropDetectionPipeline != null) {
                    spikeMarkerLocation = teamPropDetectionPipeline.getDirection();
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

        camera.stopStreaming();

        waitForStart();

        if (isStopRequested()) return;

        robot.drive.setPoseEstimate(startLocation);

        // Runs the trajectory based on the start location
        switch (spikeMarkerLocation) {
            case LEFT:
                robot.drive.followTrajectorySequenceAsync(RightNoCycle);
                break;
            case CENTER:
                robot.drive.followTrajectorySequenceAsync(CenterNoCycle);
                break;
            case RIGHT:
                robot.drive.followTrajectorySequenceAsync(LeftNoCycle);
                break;
        }

        while (opModeIsActive() && !isStopRequested()) {
            robot.update();

            telemetry.addData("Setpoint", robot.slides.getTargetPosition());
            telemetry.addData("Right Slide Motor", robot.slides.rightmotor.getCurrentPosition());
            telemetry.addData("Left Slide Motor", robot.slides.leftmotor.getCurrentPosition());
            telemetry.addData("Right Target", robot.slides.rightmotor.getTargetPosition());
            telemetry.addData("Left Target", robot.slides.leftmotor.getTargetPosition());
            telemetry.update();
        }

    }

}

package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.drive.ThreeTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.modules.statemachines.SlidesSM;
import org.firstinspires.ftc.teamcode.modules.util.SpikeMarkerLocation;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;

import java.util.List;

// Localization is doesn't show drift, follower if it does

@Config
@Autonomous(name = "Blue Close Side Auton", group = "16481-Centerstage")
public class BlueCloseAuton extends LinearOpMode{

    RobotCore robot;

    SpikeMarkerLocation spikeMarkerLocation = SpikeMarkerLocation.CENTER; // Defaults to center

    @Override
    public void runOpMode() {

        robot = new RobotCore(hardwareMap);

        Pose2d startLocation = new Pose2d(15.85, 62.00, Math.toRadians(-90));

        TrajectorySequence LeftNoCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .addDisplacementMarker(() -> {
                    robot.intake.engageLock(true,true);
                    robot.intake.flipDeposit();
                })
                .splineToLinearHeading(new Pose2d(35.00, 33.00, Math.toRadians(0)), Math.toRadians(25.74))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    robot.intake.setIntakePower(-0.3);
                })
                .waitSeconds(1.333)
                // Go to backboard
                .splineTo(new Vector2d(52.70, 40.25), Math.toRadians(0.00))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0);
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_RTP
                    );

                    robot.slides.setTargetPosition(-800);
                    robot.slides.setPower(0.8);

                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    robot.intake.clearHigherLock();
                    robot.intake.clearLowerLock();
                })
                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                    robot.intake.flipIntake();
                    robot.slides.setTargetPosition(0);
                    robot.slides.setPower(0.8);

                })
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(46.50, 43.0), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(53.43, 58.83), Math.toRadians(0.00))
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
                .splineToLinearHeading(new Pose2d(21.42, 24.03, Math.toRadians(0)), Math.toRadians(25.74))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    robot.intake.setIntakePower(-0.3);
                })
                .waitSeconds(1.333)
                // Go to backboard
                .splineTo(new Vector2d(52.70, 34.50), Math.toRadians(0.00))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0);
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_RTP
                    );

                    robot.slides.setTargetPosition(-800);
                    robot.slides.setPower(0.8);

                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    robot.intake.clearHigherLock();
                    robot.intake.clearLowerLock();
                })
                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                    robot.intake.flipIntake();
                    robot.slides.setTargetPosition(0);
                    robot.slides.setPower(0.8);

                })
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(46.50, 43.0), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(53.43, 58.83), Math.toRadians(0.00))
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
                .splineToLinearHeading(new Pose2d(18.00, 35.5, Math.toRadians(25.74)), Math.toRadians(25.74))
                .splineToConstantHeading(new Vector2d(8.00, 36.5), Math.toRadians(25.74))
                .splineToConstantHeading(new Vector2d(10.33, 36.5), Math.toRadians(25.74))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    robot.intake.setIntakePower(-0.3);
                })
                .waitSeconds(1.333)
                // Go to backboard
                .splineTo(new Vector2d(52.00, 26.00), Math.toRadians(0.00))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0);
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_RTP
                    );

                    robot.slides.setTargetPosition(-750);
                    robot.slides.setPower(0.8);

                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    robot.intake.clearHigherLock();
                    robot.intake.clearLowerLock();
                })
                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                    robot.intake.flipIntake();
                    robot.slides.setTargetPosition(0);
                    robot.slides.setPower(0.8);

                })
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(40.00, 26.00), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(53.43, 58.83), Math.toRadians(0.00))
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

        waitForStart();

        if (isStopRequested()) return;

        robot.drive.setPoseEstimate(startLocation);

        // Runs the trajectory based on the start location
        switch (spikeMarkerLocation) {
            case LEFT:
                robot.drive.followTrajectorySequenceAsync(LeftNoCycle);
                break;
            case CENTER:
                robot.drive.followTrajectorySequenceAsync(CenterNoCycle);
                break;
            case RIGHT:
                robot.drive.followTrajectorySequenceAsync(RightNoCycle);
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

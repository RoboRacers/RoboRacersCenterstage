package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.util.SpikeMarkerLocation;
import org.firstinspires.ftc.teamcode.modules.statemachines.IntakeSM;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;

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


        TrajectorySequence LeftNoCycle = robot.drive.trajectorySequenceBuilder(new Pose2d(15.85, 62.00, Math.toRadians(90)))
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


        while(!isStopRequested() && !opModeIsActive()) {
            // Vision code here
        }

        waitForStart();

        if (isStopRequested()) return;

        robot.drive.setPoseEstimate(startLocation);

        // Runs the trajectory based on the start location
        switch (spikeMarkerLocation) {
            case LEFT:
                break;
            case CENTER:
                robot.drive.followTrajectorySequence(CenterNoCycle);
            case RIGHT:
                break;
        }

    }

}

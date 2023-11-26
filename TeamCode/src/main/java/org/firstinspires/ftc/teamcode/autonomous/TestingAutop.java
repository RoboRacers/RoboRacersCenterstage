package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;

// Localization is doesn't show drift, follower if it does

@Config
@Autonomous(name = "Testing Autoop", group = "16481-Power-Play")
public class TestingAutop extends LinearOpMode{

    boolean finished = false;

    @Override
    public void runOpMode() {


        RobotCore robot = new RobotCore(hardwareMap);

        TrajectorySequence traj2 = robot.drive.trajectorySequenceBuilder(new Pose2d(12.75, -58.81, Math.toRadians(86.99)))
                .splineTo(new Vector2d(16.44, -40.18), Math.toRadians(168.93))
                .splineTo(new Vector2d(-36.13, -38.77), Math.toRadians(88.26))
                .splineTo(new Vector2d(-30.68, -11.87), Math.toRadians(-0.90))
                .splineTo(new Vector2d(43.87, -33.85), Math.toRadians(8.13))
                .build();

        TrajectorySequence RoboRPath_1_1 = robot.drive.trajectorySequenceBuilder(new Pose2d(-36.56, 62.21, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-38.19, 37.89, Math.toRadians(266.16)), Math.toRadians(266.16))
                .setReversed(false)
                .splineTo(new Vector2d(12.38, 36.11), Math.toRadians(-2.02))
                .splineTo(new Vector2d(43.38, 35.22), Math.toRadians(-1.64))
                .build();
        TrajectorySequence RoboRPath_1_2 = robot.drive.trajectorySequenceBuilder(new Pose2d(10.88, 62.78, Math.toRadians(267.61)))
                .splineTo(new Vector2d(10.88, 34.47), Math.toRadians(-88.51))
                .splineTo(new Vector2d(-59.56, 8.20), Math.toRadians(180.00))
                .splineTo(new Vector2d(-56.88, 36.29), Math.toRadians(-1.64))
                .splineTo(new Vector2d(46.48, 35.44), Math.toRadians(-1.30))
                .build();
        TrajectorySequence RoboRPath_2_1 = robot.drive.trajectorySequenceBuilder(new Pose2d(-36.83, -64.17, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-36.83, -33.19), Math.toRadians(89.36))
                .splineTo(new Vector2d(-25.25, -33.61), Math.toRadians(-14.16))
                .splineTo(new Vector2d(48.41, -37.58), Math.toRadians(2.49))
                .build();
        TrajectorySequence RoboRPath_2_2 = robot.drive.trajectorySequenceBuilder(new Pose2d(11.74, -62.14, Math.toRadians(90.00)))
                .splineTo(new Vector2d(11.10, -32.43), Math.toRadians(91.24))
                .splineTo(new Vector2d(-56.88, -9.27), Math.toRadians(164.55))
                .splineTo(new Vector2d(-58.28, -35.44), Math.toRadians(0.00))
                .splineTo(new Vector2d(49.16, -36.08), Math.toRadians(-2.47))
                .build();
        TrajectorySequence RoboBPath1_1_1 = robot.drive.trajectorySequenceBuilder(new Pose2d(-36.41, 63.25, Math.toRadians(267.36)))
                .splineTo(new Vector2d(-31.96, 31.81), Math.toRadians(-1.22))
                .splineTo(new Vector2d(-43.53, 12.83), Math.toRadians(-7.13))
                .splineTo(new Vector2d(47.83, 14.01), Math.toRadians(31.50))
                .build();
        TrajectorySequence RoboBPath1_1_2 =robot.drive.trajectorySequenceBuilder(new Pose2d(-37.00, 62.36, Math.toRadians(-86.95)))
                .splineTo(new Vector2d(-41.60, 30.92), Math.toRadians(181.40))
                .lineToSplineHeading(new Pose2d(-34.04, 8.82, Math.toRadians(4.09)))
                .splineTo(new Vector2d(50.50, 33.89), Math.toRadians(0.78))
                .build();
        TrajectorySequence RoboBPath1_1_3 = robot.drive.trajectorySequenceBuilder(new Pose2d(-35.96, 63.55, Math.toRadians(-85.35)))
                .splineTo(new Vector2d(-35.52, 34.33), Math.toRadians(266.88))
                .lineToSplineHeading(new Pose2d(50.64, 34.92, Math.toRadians(0.00)))
                .build();
        TrajectorySequence RoboBPath1_2_1 = robot.drive.trajectorySequenceBuilder(new Pose2d(11.79, 60.73, Math.toRadians(270.00)))
                .splineTo(new Vector2d(16.54, 30.33), Math.toRadians(-3.37))
                .lineToSplineHeading(new Pose2d(19.06, 8.53, Math.toRadians(20.38)))
                .splineTo(new Vector2d(50.20, 34.78), Math.toRadians(-6.71))
                .build();
        TrajectorySequence RoboBPath1_2_2 = robot.drive.trajectorySequenceBuilder(new Pose2d(11.05, 59.39, Math.toRadians(270.00)))
                .splineTo(new Vector2d(9.57, 33.00), Math.toRadians(182.91))
                .splineTo(new Vector2d(15.05, 17.87), Math.toRadians(6.79))
                .splineTo(new Vector2d(50.20, 35.67), Math.toRadians(0.00))
                .build();
        TrajectorySequence RoboBPath1_2_3 = robot.drive.trajectorySequenceBuilder(new Pose2d(12.23, 58.80, Math.toRadians(268.98)))
                .splineTo(new Vector2d(12.09, 35.81), Math.toRadians(268.15))
                .splineTo(new Vector2d(48.42, 35.52), Math.toRadians(-1.47))
                .build();
        TrajectorySequence RoboBPath2_1_1 = robot.drive.trajectorySequenceBuilder(new Pose2d(-34.04, -64.73, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-31.81, -30.92), Math.toRadians(-5.57))
                .splineTo(new Vector2d(-32.70, -6.90), Math.toRadians(-9.73))
                .splineTo(new Vector2d(15.50, -27.95), Math.toRadians(264.56))
                .splineTo(new Vector2d(50.94, -37.45), Math.toRadians(0.00))
                .build();
        TrajectorySequence RoboBPath2_1_2 = robot.drive.trajectorySequenceBuilder(new Pose2d(-35.96, -57.32, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-37.74, -30.92), Math.toRadians(178.49))
                .lineToSplineHeading(new Pose2d(-33.59, -11.64, Math.toRadians(-5.71)))
                .splineTo(new Vector2d(15.94, -20.54), Math.toRadians(-43.15))
                .splineTo(new Vector2d(50.79, -35.96), Math.toRadians(4.09))
                .build();
        TrajectorySequence RoboBPath2_1_3 = robot.drive.trajectorySequenceBuilder(new Pose2d(-36.85, -63.84, Math.toRadians(86.78)))
                .splineTo(new Vector2d(-37.45, -28.99), Math.toRadians(88.64))
                .lineToSplineHeading(new Pose2d(-29.29, -35.67, Math.toRadians(-4.40)))
                .splineTo(new Vector2d(51.24, -35.37), Math.toRadians(-2.91))
                .build();
        TrajectorySequence RoboBPath2_2_1 = robot.drive.trajectorySequenceBuilder(new Pose2d(12.83, -61.32, Math.toRadians(90.00)))
                .splineTo(new Vector2d(18.46, -32.11), Math.toRadians(0.00))
                .lineToSplineHeading(new Pose2d(15.79, -14.90, Math.toRadians(-29.62)))
                .splineTo(new Vector2d(51.09, -35.52), Math.toRadians(0.00))
                .build();
        TrajectorySequence RoboBPath2_2_2 = robot.drive.trajectorySequenceBuilder(new Pose2d(12.09, -65.47, Math.toRadians(90.00)))
                .splineTo(new Vector2d(4.67, -30.92), Math.toRadians(180.00))
                .splineTo(new Vector2d(19.50, -19.06), Math.toRadians(-71.57))
                .splineTo(new Vector2d(51.39, -36.26), Math.toRadians(0.00))
                .build();
        TrajectorySequence RoboBPath2_2_3 = robot.drive.trajectorySequenceBuilder(new Pose2d(11.94, -65.92, Math.toRadians(91.71)))
                .splineTo(new Vector2d(11.49, -26.77), Math.toRadians(90.00))
                .lineToSplineHeading(new Pose2d(17.87, -41.15, Math.toRadians(3.58)))
                .splineTo(new Vector2d(56.13, -35.52), Math.toRadians(0.00))
                .build();



        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;


        robot.drive.setPoseEstimate(RoboRPath_1_1.start());
        robot.drive.followTrajectorySequence(RoboRPath_1_1);


    }

}

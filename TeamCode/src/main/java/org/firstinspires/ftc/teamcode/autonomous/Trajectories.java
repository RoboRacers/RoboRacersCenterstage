package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.auton.TrajectorySequenceGroup;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;

public class Trajectories {

    private static RobotCore robot;

    public final Pose2d S0_POS = new Pose2d(36, -64.5, Math.toRadians(-270));
    public final Pose2d S1_POS = new Pose2d(36, 64.5, Math.toRadians(270));
    public final Pose2d S2_POS = new Pose2d(-36, 64.5, Math.toRadians(270));
    public final Pose2d S3_POS = new Pose2d(-36, -64.5, Math.toRadians(-270));

    public static void init(RobotCore initRobot) {
        // Copy over robot class.
        robot = initRobot;
    }

    /**
     * All TrajectorySequenceGroups declared from here on out.
     */

    public static class templateTraj implements TrajectorySequenceGroup {

        // Initialize trajectories and/or trajectorySequences. Remember to make them static as this class is Static and is run from a static points.
        static Trajectory traj1;
        static TrajectorySequence traj2;

        // This function is where you build your trajectories that you will use later on. Remember that your class MUST have a static init() function.
        public static void init() {
            Pose2d StartPose = new Pose2d(36, -64.5, Math.toRadians(-270));
            robot.drive.setPoseEstimate(StartPose);

            traj1 = robot.drive.trajectoryBuilder(StartPose)
                    .lineTo(new Vector2d(36, -35))
                    .build();
            traj2 = robot.drive.trajectorySequenceBuilder(traj1.end())
                    .lineTo(new Vector2d(12, -35))
                    .build();
        }

        // This function is what you will call to start the trajectories.
        public static void run() {
            robot.drive.followTrajectory(traj1);
            robot.drive.followTrajectorySequence(traj2);
        }

        // Optional; this function is to run the same trajectories asynchronously.
        public static void runAsync() {
            robot.drive.followTrajectoryAsync(traj1);
            robot.drive.followTrajectorySequenceAsync(traj2);
        }
    }

    public static class BallDropTraj implements TrajectorySequenceGroup {

        // Initialize trajectories and/or trajectorySequences. Remember to make them static as this class is Static and is run from a static points.
        static Trajectory traj1;
        static TrajectorySequence traj2;

        // This function is where you build your trajectories that you will use later on. Remember that your class MUST have a static init() function.
        public static void init() {
            Pose2d StartPose = new Pose2d(0, 0, Math.toRadians(0));
            robot.drive.setPoseEstimate(StartPose);

            traj1 = robot.drive.trajectoryBuilder(StartPose)
                    .lineTo(new Vector2d(10, 0))
                    .build();
            traj2 = robot.drive.trajectorySequenceBuilder(traj1.end())
                    .lineTo(new Vector2d(0, 0))
                    .build();
        }

        // This function is what you will call to start the trajectories.
        public static void run() {
            robot.drive.followTrajectory(traj1);
            robot.drive.followTrajectorySequence(traj2);
        }

        // Optional; this function is to run the same trajectories asynchronously.
        public static void runAsync() {
            robot.drive.followTrajectoryAsync(traj1);
            robot.drive.followTrajectorySequenceAsync(traj2);
        }
    }


}

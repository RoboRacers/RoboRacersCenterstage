package org.firstinspires.ftc.teamcode.autonomous;


import android.os.Build;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.auton.TrajectorySequenceGroup;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;

import java.util.concurrent.ThreadLocalRandom;

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
        static Trajectory traj2;
        static Trajectory traj3;
        static Trajectory traj4;
        static TrajectorySequence gobacktraj1;
        static TrajectorySequence gobacktraj2;
        static TrajectorySequence gobacktraj3;
        static TrajectorySequence gobacktraj4;

        // This function is where you build your trajectories that you will use later on. Remember that your class MUST have a static init() function.
        public static void init() {
            Pose2d StartPose = new Pose2d(0, 0, Math.toRadians(0));
            robot.drive.setPoseEstimate(StartPose);

            traj1 = robot.drive.trajectoryBuilder(StartPose)
                    .lineTo(new Vector2d(20, 20))
                    .build();
            traj2 = robot.drive.trajectoryBuilder(StartPose)
                    .lineTo(new Vector2d(20, -20))
                    .build();
            traj3 = robot.drive.trajectoryBuilder(StartPose)
                    .lineTo(new Vector2d(-20, 20))
                    .build();
            traj4 = robot.drive.trajectoryBuilder(StartPose)
                    .lineTo(new Vector2d(-20, -20))
                    .build();
            gobacktraj1 = robot.drive.trajectorySequenceBuilder(traj1.end())
                    .lineTo(new Vector2d(0, 0))
                    .build();
            gobacktraj2 = robot.drive.trajectorySequenceBuilder(traj2.end())
                    .lineTo(new Vector2d(0, 0))
                    .build();
            gobacktraj3 = robot.drive.trajectorySequenceBuilder(traj3.end())
                    .lineTo(new Vector2d(0, 0))
                    .build();
            gobacktraj4 = robot.drive.trajectorySequenceBuilder(traj4.end())
                    .lineTo(new Vector2d(0, 0))
                    .build();
        }

        // This function is what you will call to start the trajectories.
        public static void run() {
            int whichTraj = 1;
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
                whichTraj = ThreadLocalRandom.current().nextInt(1,4);
            }
            switch (whichTraj) {
                case 1:
                    robot.drive.followTrajectory(traj1);
                    robot.drive.followTrajectorySequence(gobacktraj1);
                case 2:
                    robot.drive.followTrajectory(traj2);
                    robot.drive.followTrajectorySequence(gobacktraj2);
                case 3:
                    robot.drive.followTrajectory(traj3);
                    robot.drive.followTrajectorySequence(gobacktraj3);
                case 4:
                    robot.drive.followTrajectory(traj4);
                    robot.drive.followTrajectorySequence(gobacktraj4);
            }
        }

        // Optional; this function is to run the same trajectories asynchronously.
        public static void runAsync() {
            robot.drive.followTrajectoryAsync(traj1);
        }
    }


}

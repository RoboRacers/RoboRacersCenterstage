package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.auton.TrajectorySequenceGroup;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;

public class Trajectories {

    private static RobotCore robot;
    private static Telemetry telemetry;

    public final Pose2d S0_POS = new Pose2d(36, -64.5, Math.toRadians(-270));
    public final Pose2d S1_POS = new Pose2d(36, 64.5, Math.toRadians(270));
    public final Pose2d S2_POS = new Pose2d(-36, 64.5, Math.toRadians(270));
    public final Pose2d S3_POS = new Pose2d(-36, -64.5, Math.toRadians(-270));

    public static void init(RobotCore initRobot, Telemetry initTelemetry) {
        // Copy over robot class.
        robot = initRobot;
        telemetry = initTelemetry;
    }

    /**
     * All TrajectorySequenceGroups declared from here on out.
     */

    public static class templateTraj implements TrajectorySequenceGroup {

        // Initialize trajectories and/or trajectorySequences. Remember to make them static as this class is Static and is run from a static points.
        static Trajectory traj1;

        // This function is where you build your trajectories that you will use later on. Remember that your class MUST have a static init() function.
        public static void init() {
            Pose2d StartPose = new Pose2d(0, 0, Math.toRadians(0));
            robot.drive.setPoseEstimate(StartPose);

            traj1 = robot.drive.trajectoryBuilder(StartPose)
                    .splineTo(new Vector2d(20,20), Math.toRadians(90))
                    .addDisplacementMarker(() -> {
                        robot.setArmPos(600, 0.5);
                    })
                    .build();
        }

        // This function is what you will call to start the trajectories.
        public static void run() {
            robot.drive.followTrajectory(traj1);
        }

        // Optional; this function is to run the same trajectories asynchronously.
        public static void runAsync() {
            robot.drive.followTrajectoryAsync(traj1);
        }
    }


}

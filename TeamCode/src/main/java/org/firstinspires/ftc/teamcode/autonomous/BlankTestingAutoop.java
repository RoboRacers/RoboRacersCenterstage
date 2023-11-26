package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.statemachines.IntakeSM;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;

// Localization is doesn't show drift, follower if it does

@Config
@Autonomous(name = "Blank Testing Autoop", group = "16481-Power-Play")
public class BlankTestingAutoop extends LinearOpMode{

    RobotCore robot;

    @Override
    public void runOpMode() {

        robot = new RobotCore(hardwareMap);

        TrajectorySequence traj1 = robot.drive.trajectorySequenceBuilder(new Pose2d(15.85, 62.00, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(12.95, 29.04), Math.toRadians(266.31))
                .setReversed(false)
                .splineTo(new Vector2d(7.46, 40.02), Math.toRadians(143.13))
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    robot.intake.statemachine.transition(IntakeSM.EVENT.EXTEND_WITH_PIXEL);
                    robot.slides.setTargetPosition(800);
                    robot.slides.setPower(.8);
                })
                .splineTo(new Vector2d(46.50, 35.90), Math.toRadians(0.00))
                .addDisplacementMarker(() -> {
                    robot.intake.statemachine.transition(IntakeSM.EVENT.RELEASE_PIXEL);
                    robot.slides.setTargetPosition(0);
                })
                .setReversed(false)
                .splineTo(new Vector2d(39, 35.90), Math.toRadians(180))
                .setReversed(true)
                .splineTo(new Vector2d(58,60), Math.toRadians(0))
                .build();

        TrajectorySequence traj2 = robot.drive.trajectorySequenceBuilder(new Pose2d(15.85, 62.00, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(11.56, 44.15), Math.toRadians(270.00))
                .splineTo(new Vector2d(2.27, 31.90), Math.toRadians(232.84))
                .setReversed(false)
                .splineTo(new Vector2d(14.32, 44.35), Math.toRadians(45.92))
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    robot.intake.statemachine.transition(IntakeSM.EVENT.EXTEND_WITH_PIXEL);
                    robot.slides.setTargetPosition(800);
                    robot.slides.setPower(.8);
                })
                .splineTo(new Vector2d(46.50, 35.90), Math.toRadians(0.00))
                .addDisplacementMarker(() -> {
                    robot.intake.statemachine.transition(IntakeSM.EVENT.RELEASE_PIXEL);
                    robot.slides.setTargetPosition(0);
                })
                .setReversed(false)
                .splineTo(new Vector2d(39, 35.90), Math.toRadians(180))
                .setReversed(true)
                .splineTo(new Vector2d(58,60), Math.toRadians(0))
                .build();








        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;


        robot.drive.setPoseEstimate(traj1.start());
        robot.drive.followTrajectorySequence(traj1);


    }

}

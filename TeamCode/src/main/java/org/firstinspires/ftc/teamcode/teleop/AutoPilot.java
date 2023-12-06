package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;

@TeleOp(name = "AutoPilot", group = "16481-Centerstage")
public class AutoPilot extends LinearOpMode {

    RobotCore robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotCore(hardwareMap);
        TrajectorySequence toBlueBackBoard = robot.drive.trajectorySequenceBuilder(new Pose2d(-58.93, -58.37, Math.toRadians(223.53)))
                .splineTo(new Vector2d(-41.62, -12.71), Math.toRadians(69.24))
                .splineTo(new Vector2d(11.60, 7.18), Math.toRadians(90.00))
                .splineTo(new Vector2d(43.83, 35.54), Math.toRadians(-4.76))
                .build();

        TrajectorySequence toRedBackBoard = robot.drive.trajectorySequenceBuilder(new Pose2d(-58.93, 58.37, Math.toRadians(136.47)))
                .splineTo(new Vector2d(-41.62, 12.71), Math.toRadians(290.76))
                .splineTo(new Vector2d(11.60, -7.18), Math.toRadians(270.00))
                .splineTo(new Vector2d(49.26, -38.73), Math.toRadians(-4.76))
                .build();


        while (opModeInInit()) {
        }

        while (!isStopRequested()) {


        }
    }
}

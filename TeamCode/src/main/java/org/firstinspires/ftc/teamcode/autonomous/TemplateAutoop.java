package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.modules.drive.MecanumDrive.FollowTrajectoryAction;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

// Localization is doesn't show drift, follower if it does

@Config
@Autonomous(name = "AutoOp State Machines", group = "16481-Power-Play")
public class TemplateAutoop extends LinearOpMode {

    public static boolean finished = false;

    static ElapsedTime time;

    @Override
    public void runOpMode() {

        RobotCore robot = new RobotCore(hardwareMap);

        FollowTrajectoryAction traj1 = (FollowTrajectoryAction) robot.drive.actionBuilder(robot.drive.pose)
                .splineTo(new Vector2d(15, 15), Math.toRadians(90))
                .build();


        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                traj1
        );

    }

}

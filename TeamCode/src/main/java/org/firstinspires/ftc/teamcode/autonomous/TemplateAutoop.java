package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.drive.MecanumDrive.FollowTrajectoryAction;
import org.firstinspires.ftc.teamcode.modules.drive.MecanumDrive;
// Localization is doesn't show drift, follower if it does

@Config
@Autonomous(name = "AutoOp State Machines", group = "16481-Power-Play")
public class TemplateAutoop extends LinearOpMode {

    boolean finished = false;

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        DcMotor motor1 = hardwareMap.get(DcMotor.class,  "motor");

        // Delcare Trajectory as such
        Action TrajectoryAction1 = drive.actionBuilder(drive.pose)
                .lineToX(0)
                .build();

        // Bit more complicated way of doing it if you want access to the data of the trajectory
        FollowTrajectoryAction TrajectoryAction2 = (FollowTrajectoryAction) drive.actionBuilder(new Pose2d(0,0,0))
                .splineTo(new Vector2d(5,5), Math.toRadians(90))
                .build();


        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        TrajectoryAction1, // Example of a drive action

                        // This actions and the following action do the same thing
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                telemetry.addLine("Action!");
                                return false;
                            }
                        },
                        // Only that this action uses a Lambda expression to reduce complexity
                        (telemetryPacket) -> {
                            telemetry.addLine("Action!");
                            return false; // Returning false causes the action to run again, returning false causes it to cease
                        },
                        new ParallelAction( // several actions being run in parallel
                                TrajectoryAction2,
                                (telemetryPacket) -> {
                                    motor1.setPower(1);
                                    return false;
                                }
                        )

                )
        );


    }

}

package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.drive.MecanumDrive;

public class TestAuton {
    MecanumDrive drive;


    TestAuton(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        Action traj1 = drive.actionBuilder(drive.pose)
                .lineToX(0)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        traj1,
                        telemetryPacket -> {

                            return false;
                        }
                )
        );

    }

    public static void main(String[] args) {
        System.out.println("Sucess!");
    }

}

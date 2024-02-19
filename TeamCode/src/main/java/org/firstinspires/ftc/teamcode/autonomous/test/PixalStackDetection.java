package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;

// Localization if it shows drift, follower if it doesn't
// - Words to code by

@Config
@Autonomous(name = "Pixal Stack", group = "16481-Template")
//@Disabled /* COMMENT THIS OUT WHEN YOU CREATE A NEW OPMODE */
public class PixalStackDetection extends LinearOpMode{

    private HuskyLens huskyLens;
    RobotCore robot;

    @Override
    public void runOpMode() {


        robot = new RobotCore(hardwareMap);
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        TrajectorySequence traj2 = robot.drive.trajectorySequenceBuilder(new Pose2d(0, 0))
                .addDisplacementMarker(() -> {

                })
                .build();




        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;
        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);

        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block", blocks[i].toString());


        }


        telemetry.update();

        robot.drive.followTrajectorySequenceAsync(traj2);

    }

}

package org.firstinspires.ftc.teamcode.autonomous;

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
@Autonomous(name = "Template Autoop", group = "16481-Template")
//@Disabled /* COMMENT THIS OUT WHEN YOU CREATE A NEW OPMODE */
public class MoveToYellowPixal extends LinearOpMode{

    private HuskyLens huskyLens;
    RobotCore robot;
    Pose2d scanPoint;

    Pose2d yellowPixal;
    double yellowPixalX;

    double yVal;

    @Override
    public void runOpMode() {


        robot = new RobotCore(hardwareMap);
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        scanPoint = new Pose2d(12.75, -58.81, Math.toRadians(86.99));

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        TrajectorySequence traj2 = robot.drive.trajectorySequenceBuilder(scanPoint)
                .addDisplacementMarker(() -> {


                })
                .build();

        TrajectorySequence moveLeft = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .addDisplacementMarker(() -> {
                    yVal = robot.drive.getPoseEstimate().getY() + 1.0;
        })

                .lineToConstantHeading(new Vector2d(robot.drive.getPoseEstimate().getX(), yVal))
                .build();



        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;
        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);


        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block", blocks[i].toString());
            yellowPixalX = blocks[i].x; //add if condition so only yellow is detected
            while(yellowPixalX != 160){
                if(yellowPixalX > 160){

                }else if(yellowPixalX < 160){

                }
            }
        }


        telemetry.update();

        robot.drive.followTrajectorySequenceAsync(traj2);

    }

}

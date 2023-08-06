package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotCore;

// Localization is doesn't show drift, follower if it does

@Config
@Autonomous(name = "Ball Throw Autoop", group = "Test")
public class BallThrowAutoop extends LinearOpMode {

    boolean finished = false;

    @Override
    public void runOpMode() {

        RobotCore robot = new RobotCore(hardwareMap);

        while(!isStopRequested() && !opModeIsActive()) {

        }

        // Init
        Trajectories.init(robot);
        Trajectories.BallDropTraj.init();

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive() && !finished){
            Trajectories.BallDropTraj.run();
        }

    }

}

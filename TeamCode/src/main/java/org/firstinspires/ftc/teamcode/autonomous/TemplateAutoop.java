/*
package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotCore;

// Localization is doesn't show drift, follower if it does

@Config
@Autonomous(name = "AutoOp State Machines", group = "16481-Power-Play")
public class TemplateAutoop extends LinearOpMode {

    boolean finished = false;

    @Override
    public void runOpMode() {

        RobotCore robot = new RobotCore(hardwareMap, gamepad1, gamepad2);
        robot.initOpenCV();

        Trajectories.init(robot, telemetry);

        // Init
        Trajectories.templateTraj.init();

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive() && !finished){
            Trajectories.templateTraj.run();
            finished = true;
        }

    }

}
*/
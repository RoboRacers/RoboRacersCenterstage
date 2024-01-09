package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.RobotCore;


@TeleOp(name = "Axon Programming Test", group = "16481-Template")
public class AxonProgrammingTest extends LinearOpMode {
    RobotCore robot;


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotCore(hardwareMap);


        while (opModeInInit()) {

        }

        while (!isStopRequested()) {

            telemetry.update();
            robot.update();
        }
    }
}

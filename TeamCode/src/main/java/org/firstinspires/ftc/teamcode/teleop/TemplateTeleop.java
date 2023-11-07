package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.onbotjava.handlers.objbuild.WaitForBuild;
import org.firstinspires.ftc.teamcode.RobotCore;

@TeleOp(name = "Template Teleop", group = "16481-Centerstage")
public class TemplateTeleop extends LinearOpMode {

    RobotCore robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotCore(hardwareMap);

        while (opModeInInit()) {

        }


        while (!isStopRequested()) {

            robot.drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    gamepad1.right_stick_x
                            ),
                            -gamepad1.right_stick_x
                    )
            );

            Pose2d robotPose = robot.drive.pose;
        }
    }
}

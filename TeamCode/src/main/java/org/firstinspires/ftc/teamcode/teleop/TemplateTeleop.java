package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;

@TeleOp(name = "Template Teleop", group = "Test")
public class TemplateTeleop extends LinearOpMode {

    double driveSensitivity = .5;
    double turnSensitivity = .75;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotCore robot = new RobotCore(hardwareMap);

        while (opModeInInit()) {
        }

        while (!isStopRequested()) {

            robot.drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y*driveSensitivity, -gamepad1.left_stick_x*driveSensitivity, -gamepad1.right_stick_x*turnSensitivity));
            robot.drive.update();

        }
    }
}

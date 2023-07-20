package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.localization.MonteCarloLocalizer;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.localization.MonteCarloLocalizerTest;

import java.util.List;

@TeleOp(name = "MCL Localization Teleop", group = "Test")
public class TemplateTeleop extends LinearOpMode {

    double driveSensitivity = .5;
    double turnSensitivity = .75;

    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotCore robot = new RobotCore(hardwareMap);
        MonteCarloLocalizer mcl = new MonteCarloLocalizer(hardwareMap);

        dashboard = FtcDashboard.getInstance();

        long loop;
        long loopTime = 0;

        List<Pose2d> poses;

        while (opModeInInit()) {

        }

        while (!isStopRequested()) {
            mcl.update();

            robot.drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y*driveSensitivity,-gamepad1.left_stick_x*driveSensitivity, -gamepad1.right_stick_x*turnSensitivity));
            robot.drive.update();

            poses = mcl.getParticlePoses();

            /*
            for (Pose2d pose: poses) {
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().strokeCircle(pose.getX(), pose.getY(), 0.5).strokeCircle();
                dashboard.sendTelemetryPacket(packet);
            }

             */

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().strokeCircle(poses.get(1).getX(), poses.get(1).getY(), 0.2)
                    .strokeCircle(poses.get(2).getX(), poses.get(2).getY(), 0.2)
                    .strokeCircle(poses.get(3).getX(), poses.get(3).getY(), 0.2)
                    .strokeCircle(poses.get(4).getX(), poses.get(4).getY(), 0.2)
                    .strokeCircle(poses.get(5).getX(), poses.get(5).getY(), 0.2)
                    .strokeCircle(poses.get(6).getX(), poses.get(6).getY(), 0.2)
                    .strokeCircle(poses.get(7).getX(), poses.get(7).getY(), 0.2)
                    .strokeCircle(poses.get(8).getX(), poses.get(8).getY(), 0.2)
                    .strokeCircle(poses.get(9).getX(), poses.get(9).getY(), 0.2)
                    .strokeCircle(poses.get(10).getX(), poses.get(10).getY(), 0.2)
                    .strokeCircle(poses.get(11).getX(), poses.get(11).getY(), 0.2)
                    .strokeCircle(poses.get(12).getX(), poses.get(12).getY(), 0.2)
                    .strokeCircle(poses.get(13).getX(), poses.get(13).getY(), 0.2)
                    .strokeCircle(poses.get(14).getX(), poses.get(14).getY(), 0.2);
            dashboard.sendTelemetryPacket(packet);

            loop = System.nanoTime();
            telemetry.addData("hz", 1000000000/(loop-loopTime));
            telemetry.update();
            loopTime = loop;

        }
    }
}

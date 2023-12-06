package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboracers.gaeldrive.particles.Particle;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.localization.MonteCarloLocalizer;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

@TeleOp(name = "Field Centric Teleop", group = "Test")
public class FieldCentricTest extends LinearOpMode {

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
        ArrayList<Particle> map;

        while (opModeInInit()) {

        }

        while (!isStopRequested()) {
            mcl.update();

            double robotAngle = FastMath.atan2(-gamepad1.left_stick_x*driveSensitivity,-gamepad1.left_stick_y*driveSensitivity);
            double robotHeading = robot.drive.getPoseEstimate().getHeading();




            robot.drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y*driveSensitivity,-gamepad1.left_stick_x*driveSensitivity, -gamepad1.right_stick_x*turnSensitivity));
            robot.drive.update();

            poses = mcl.getParticlePoses();






            TelemetryPacket packet = new TelemetryPacket();
            for (Pose2d pose: poses) {
                packet.fieldOverlay().strokeCircle(pose.getX(), pose.getY(), 0.5);
            }
            dashboard.sendTelemetryPacket(packet);

            loop = System.nanoTime();
            telemetry.addData("hz", 1000000000/(loop-loopTime));
            telemetry.update();
            loopTime = loop;

        }
    }
}

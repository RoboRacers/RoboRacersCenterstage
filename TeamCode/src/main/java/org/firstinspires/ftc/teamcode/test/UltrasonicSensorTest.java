package org.firstinspires.ftc.teamcode.test;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.modules.util.UltrasonicDistanceSensor;

import java.util.List;


@TeleOp(name = "Ultrasonic Sensor Test", group = "Test")
public class UltrasonicSensorTest extends LinearOpMode {

    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        while (!isStopRequested()) {

            // Telemetry
            telemetry.addLine("Ultrasonic Testing");
            telemetry.addData("Left Ultrasonic", drive.leftUltrasonic.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Ultrasonic", drive.rightUltrasonic.getDistance(DistanceUnit.INCH));
            telemetry.update();

        }
    }
}

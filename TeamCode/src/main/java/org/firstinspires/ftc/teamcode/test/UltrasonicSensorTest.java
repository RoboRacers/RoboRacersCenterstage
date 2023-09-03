package org.firstinspires.ftc.teamcode.test;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;

import org.firstinspires.ftc.teamcode.RobotCore;

import java.util.List;

@TeleOp(name = "Ultrasonic Sensor Test", group = "Test")
public class UltrasonicSensorTest extends LinearOpMode {

    AnalogInput ultrasonicSensor;


    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        ultrasonicSensor = hardwareMap.get(AnalogInput.class, "ultrasonic1");

        KalmanFilter kalmanFilter = new KalmanFilter(0.7, 2,3 );
        LowPassFilter lowPassFilter = new LowPassFilter(0.5);

        while (opModeInInit()) {
        }

        double range = 0;
        double kalmanRange;
        double lowPassRange;

        while (!isStopRequested()) {

            range = (ultrasonicSensor.getVoltage() * 312.5)/2.54;
            kalmanRange = kalmanFilter.estimate(range);
            lowPassRange = lowPassFilter.estimate(range);

            // Telemetry
            telemetry.addData("Ultrasonic Testing", "");
            telemetry.addData("Raw Sensor Range (Inches)", range);
            telemetry.addData("Kalman Sensor Range (Inches)", kalmanRange);
            telemetry.addData("Low Pass Sensor Range (Inches)", lowPassRange);
            telemetry.update();

        }
    }
}

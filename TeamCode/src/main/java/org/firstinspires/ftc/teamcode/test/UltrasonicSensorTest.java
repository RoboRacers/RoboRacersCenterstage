package org.firstinspires.ftc.teamcode.test;
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


        while (opModeInInit()) {
        }

        double range = 0;
        double kalmanRange;
        double lowPassRange;

        while (!isStopRequested()) {

            range = (ultrasonicSensor.getVoltage() * 312.5)/2.54;
            // Telemetry
            telemetry.addData("Ultrasonic Testing", "");
            telemetry.addData("Raw Sensor Range (Inches)", range);
            telemetry.update();

        }
    }
}

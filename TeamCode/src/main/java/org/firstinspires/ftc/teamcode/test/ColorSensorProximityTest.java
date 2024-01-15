package org.firstinspires.ftc.teamcode.test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Color Sensor Proximity Test", group = "Test")
public class ColorSensorProximityTest extends LinearOpMode {

    public ColorSensor colorSensor;
    @Override
    public void runOpMode() throws InterruptedException {

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        waitForStart();

        while (!isStopRequested()) {

            telemetry.addData("Alpha", colorSensor.alpha());
            telemetry.addData("ARGB", colorSensor.argb());
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.update();

        }
    }
}



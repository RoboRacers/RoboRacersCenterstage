package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp(name = "Rev 2m Sensor Test", group = "Test")
public class Rev2mSensorTest extends LinearOpMode {

    Rev2mDistanceSensor rev2mDistanceSensor;


    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        rev2mDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rev2m1");


        while (opModeInInit()) {
        }

        double range;
        double kalmanRange;
        double lowPassRange;

        while (!isStopRequested()) {


            range = rev2mDistanceSensor.getDistance(DistanceUnit.INCH);

            // Telemetry
            telemetry.addData("Rev2m Testing", "");
            telemetry.addData("Raw Sensor Range (Inches)", range);
            telemetry.update();

        }
    }
}

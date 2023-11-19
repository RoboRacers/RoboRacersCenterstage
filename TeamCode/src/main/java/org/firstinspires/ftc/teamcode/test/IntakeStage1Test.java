package org.firstinspires.ftc.teamcode.test;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp(name = "Intake Stage 1 Test", group = "Test")
public class IntakeStage1Test extends LinearOpMode {

    Servo rightStage1;
    ServoImplEx leftStage1;

    @Override
    public void runOpMode() throws InterruptedException {

        rightStage1 = hardwareMap.get(Servo.class, "rightStage1");
        //leftStage1 = hardwareMap.get(ServoImplEx.class, "leftStage1");



        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }




        while (opModeInInit()) {
        }


        //leftStage1.setPosition(0.5);

        while (!isStopRequested()) {

            // Telemetry
            telemetry.addData("Right Servo Position", "");
            //telemetry.addData("Left Servo Position", leftStage1.getPosition());
            telemetry.update();

        }
    }
}

package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.subsystems.Vision;
import org.firstinspires.ftc.teamcode.modules.util.SpikeMarkerLocation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name="Vision Subsystem Test", group="Test")
public class VisionSubsystemTest extends LinearOpMode {
    private Vision teamVisionPipeline;

    @Override
    public void runOpMode() {

        teamVisionPipeline = new Vision(hardwareMap);
        teamVisionPipeline.startPropDetection();

        while (opModeInInit() && !isStopRequested()) {
            telemetry.addData("Camera:", teamVisionPipeline.cameraOpened);
            telemetry.update();
        }

        waitForStart();


        while (opModeIsActive()) {
            SpikeMarkerLocation direction = teamVisionPipeline.getDirection();

            telemetry.addData("Direction", direction);
            telemetry.addData("Opened", teamVisionPipeline.cameraOpened);
            telemetry.update();
        }

        //camera.closeCameraDevice();
    }
}
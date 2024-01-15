package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.subsystems.Vision;
import org.firstinspires.ftc.teamcode.modules.util.SpikeMarkerLocation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name="Team Prop Detection OpMode", group="Test")
public class OpenCVPropDetection extends LinearOpMode {
    private OpenCvWebcam camera;
    private Vision teamVisionPipeline;

    @Override
    public void runOpMode() {

        teamVisionPipeline = new Vision(hardwareMap);
        teamVisionPipeline.startPropDetection();
        waitForStart();

        while (opModeIsActive()) {
            SpikeMarkerLocation direction = teamVisionPipeline.getDirection();

            telemetry.addData("Direction", direction);
            telemetry.addData("Opened",teamVisionPipeline.cameraOpened);
            telemetry.update();
        }

        //camera.closeCameraDevice();
    }
}
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.modules.subsystems.Vision;
import org.firstinspires.ftc.teamcode.modules.util.SpikeMarkerLocation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="Vision Simple Test", group="Test")
public class VisionSimpleTest extends LinearOpMode {

    OpenCvCamera camera;

    String cameraname = "Webcam 1";

    public Vision.TeamPropPipeline teamPropDetectionPipeline = null;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap
                .get(WebcamName.class, cameraname), cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                teamPropDetectionPipeline = new Vision.TeamPropPipeline();
                camera.setPipeline(teamPropDetectionPipeline);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (opModeInInit() && !isStopRequested()) {
            telemetry.addData("Camera:",camera);
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive()) {
            SpikeMarkerLocation direction = null;
            if (teamPropDetectionPipeline != null) {
                direction = teamPropDetectionPipeline.getDirection();
            }
            telemetry.addData("Direction", direction);
            telemetry.update();
        }

    }
}
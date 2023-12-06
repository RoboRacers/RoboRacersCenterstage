package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.modules.subsystems.PropDetection;
import org.firstinspires.ftc.teamcode.util.SpikeMarkerLocation;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name="Team Prop Detection OpMode", group="Test")
public class OpenCVPropDetection extends LinearOpMode {
    private OpenCvWebcam camera;
    private PropDetection teamPropDetectionPipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap
                .get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        teamPropDetectionPipeline = new PropDetection(camera, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            SpikeMarkerLocation direction = teamPropDetectionPipeline.getDirection();

            telemetry.addData("Direction", direction);
            telemetry.update();
        }

        //camera.closeCameraDevice();
    }
}
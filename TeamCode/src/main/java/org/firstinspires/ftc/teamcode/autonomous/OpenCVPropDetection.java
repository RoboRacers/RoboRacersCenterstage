package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name="Team Prop Detection OpMode", group="Linear Opmode")
public class OpenCVPropDetection extends LinearOpMode {
    private OpenCvWebcam camera;
    private TeamPropDetectionPipeline teamPropDetectionPipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap
                .get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        teamPropDetectionPipeline = new TeamPropDetectionPipeline(camera, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            String direction = teamPropDetectionPipeline.getDirection();

            telemetry.addData("Direction", direction);
            telemetry.update();
        }

        //camera.closeCameraDevice();
    }
}
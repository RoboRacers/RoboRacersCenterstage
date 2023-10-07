package org.firstinspires.ftc.teamcode.modules.opencv;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name="Pixel Detect OpMode", group="Linear Opmode")
public class PixelDetectOpmode extends LinearOpMode {

    @Override
    public void runOpMode() {
        OpenCvWebcam camera;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.
                get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        PixelDetection myPixelDetection = new PixelDetection(camera, telemetry);

        double pixelCenterX = 0.0;
        double pixelCenterY = 0.0;

        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            pixelCenterX = myPixelDetection.getCenterX();
            pixelCenterY = myPixelDetection.getCenterY();

            int DisplayCenterX = 320;
            int DisplayCenterY = 240;

            String directionX = "NAX";
            String directionY = "NAY";

            if (pixelCenterX > 0 && pixelCenterX < DisplayCenterX) {
                directionX = "left";
            } else  {
                directionX = "right";
            }

            if (pixelCenterY > 0 && pixelCenterY < DisplayCenterY) {
                directionY = "up";
            } else  {
                directionY = "down";
            }

            telemetry.addData(directionX, directionY);

            telemetry.addData("Pixel Center X = ", pixelCenterX);
            telemetry.addData("Pixel Center Y = ", pixelCenterY);
            telemetry.update();

        }
    }
}

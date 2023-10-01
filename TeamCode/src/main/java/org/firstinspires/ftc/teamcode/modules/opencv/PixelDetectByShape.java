package org.firstinspires.ftc.teamcode.modules.opencv;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PixelDetectByShape {
    PixelDetectionPipeline pixelDetectionPipeline;
    Telemetry telemetry;

    public PixelDetectByShape(OpenCvCamera camera, Telemetry telemetry) {
        this.telemetry = telemetry;
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                pixelDetectionPipeline = new PixelDetectionPipeline();
                camera.setPipeline(pixelDetectionPipeline);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (pixelDetectionPipeline == null) {
        }
    }

    public double getCenterX() {
        return pixelDetectionPipeline.getCenterX();
    }

    public double getCenterY() {
        return pixelDetectionPipeline.getCenterY();
    }

    static class PixelDetectionPipeline extends OpenCvPipeline {
        private double centerX = 0.0;
        private double centerY = 0.0;

        @Override
        public Mat processFrame(Mat frame) {
            Mat gray = new Mat();
            Imgproc.cvtColor(frame, gray, Imgproc.COLOR_BGR2GRAY);

            Mat edges = new Mat();
            Imgproc.Canny(gray, edges, 300, 900);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(edges, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                Moments M = Imgproc.moments(contour);

                if (M.get_m00() != 0) {
                    centerX = M.get_m10() / M.get_m00();
                    centerY = M.get_m01() / M.get_m00();
                } else {
                    // Handle the case where the contour has no area
                    centerX = 0.0;
                    centerY = 0.0;
                }
            }

            // Return the processed frame (not really needed for your purpose)
            return frame;
        }

        public double getCenterX() {
            return centerX;
        }

        public double getCenterY() {
            return centerY;
        }
    }
}

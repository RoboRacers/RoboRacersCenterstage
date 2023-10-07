package org.firstinspires.ftc.teamcode.modules.opencv;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PixelDetectionByShape {
    PixelDetectionPipeline pixelDetectionPipeline;
    Telemetry telemetry;

    public PixelDetectionByShape(OpenCvCamera camera, Telemetry telemetry) {
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

            Imgproc.medianBlur(gray, gray, 5);

            Imgproc.adaptiveThreshold(gray, gray, 255, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY, 11, 2);

            Mat edges = new Mat();
            Imgproc.Canny(gray, edges, 300, 900);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(edges, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            List<MatOfPoint> hexContours = new ArrayList<>();

            for(MatOfPoint aContour:contours) {
                //draw
                Scalar scalar = new Scalar(0,128,150,255);
                Imgproc.drawContours(gray, contours, -1, scalar, 5);

                //get perimeter of contour
                MatOfPoint2f newC = new MatOfPoint2f( aContour.toArray() );
                double perimeter = Imgproc.arcLength(newC, true);

                MatOfPoint2f approx = new MatOfPoint2f();
                Imgproc.approxPolyDP(newC, approx, 0.05, true);

                if(approx.size().height == 6) {
                    //found hexagon
                    hexContours.add(aContour);
                }
            }

            //find max area contour from hexContours

            double maxVal = 0;
            int maxValIdx = 0;
            for (int contourIdx = 0; contourIdx < hexContours.size(); contourIdx++)
            {
                double contourArea = Imgproc.contourArea(hexContours.get(contourIdx));
                if (maxVal < contourArea)
                {
                    maxVal = contourArea;
                    maxValIdx = contourIdx;
                }
            }

            //draw contour with index maxValIdx
            Imgproc.drawContours(frame, hexContours, maxValIdx, new Scalar(0,255,0), 5);


            Moments M = Imgproc.moments(hexContours.get(maxValIdx));
            if (M.get_m00() != 0) {
                centerX = M.get_m10() / M.get_m00();
                centerY = M.get_m01() / M.get_m00();
            } else {
                // Handle the case where the contour has no area
                centerX = 0.0;
                centerY = 0.0;
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

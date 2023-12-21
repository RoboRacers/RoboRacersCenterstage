package org.firstinspires.ftc.teamcode.modules.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.SpikeMarkerLocation;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class Vision extends Subsystem {
    public TeamPropPipeline teamPropDetectionPipeline;
    Telemetry telemetry;

    public Vision(HardwareMap hardwareMap) {

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap
                .get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                teamPropDetectionPipeline = new TeamPropPipeline();
                camera.setPipeline(teamPropDetectionPipeline);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        //while (teamPropDetectionPipeline == null) {
        //}
    }

    public SpikeMarkerLocation getDirection() {
        if (teamPropDetectionPipeline != null) {
            return teamPropDetectionPipeline.getDirection();
        } else {
            return null;
        }

    }
    public String getElapsedTime() {
        return teamPropDetectionPipeline.getElapsedTime();
    }
    public String getEndTime() {
        return teamPropDetectionPipeline.getEndTime();
    }

    @Override
    public void update() {

    }


    static class TeamPropPipeline extends OpenCvPipeline {

        Mat gray = new Mat();
        Mat shadowMask = new Mat();
        Mat labImage = new Mat();
        List<Mat> labChannels = new ArrayList<>();
        Mat noShadow = new Mat();
        Mat firstGray = new Mat();
        Mat binaryImg = new Mat();
        Mat blackCountImg = new Mat();

        int frameCount = 1;


        private SpikeMarkerLocation direction = SpikeMarkerLocation.CENTER;

        private String elapsedTime = "";

        private String endElapsedTime = "";

        @Override
        public Mat processFrame(Mat frame) {

            long startTime = System.currentTimeMillis();

            if (frameCount == 4) {


                Imgproc.cvtColor(frame, gray, Imgproc.COLOR_BGR2GRAY);

                Imgproc.medianBlur(gray, gray, 15);

                // Apply adaptive thresholding to identify shadows
                Imgproc.adaptiveThreshold(gray, shadowMask, 0, Imgproc.ADAPTIVE_THRESH_MEAN_C, Imgproc.THRESH_BINARY, 11, 10);

                // Invert the shadow mask
                Core.bitwise_not(shadowMask, shadowMask);

                // Convert the original image to LAB color space
                Imgproc.cvtColor(frame, labImage, Imgproc.COLOR_BGR2Lab);

                // Split LAB image into L, A, and B channels
                Core.split(labImage, labChannels);

                // Replace L channel with the shadow-masked L channel
                labChannels.set(0, shadowMask);

                // Merge LAB channels back into one image
                Core.merge(labChannels, labImage);

                // Convert LAB image back to BGR
                Imgproc.cvtColor(labImage, noShadow, Imgproc.COLOR_Lab2BGR);


                //firstGray = noShadow.clone();

                Imgproc.cvtColor(noShadow, firstGray, Imgproc.COLOR_BGR2GRAY);
                Imgproc.threshold(firstGray, blackCountImg, 0, 255, Imgproc.THRESH_BINARY + Imgproc.THRESH_OTSU);

                long endTime = System.currentTimeMillis();

                elapsedTime = ""+(endTime - startTime);


                //blackCountImg = binaryImg.clone();

                int leftImageWidth = blackCountImg.cols() / 3;
                int centerImageWidth = 2 * (leftImageWidth);
                int rightImageWidth = blackCountImg.cols();

                int leftBlackPixelCount = 0;
                int centerBlackPixelCount = 0;
                int rightBlackPixelCount = 0;

                /*
                for (int row = 0; row < blackCountImg.rows(); row++) {
                    for (int col = 0; col < leftImageWidth; col++) {
                        double pixelValue = blackCountImg.get(row, col)[0];
                        if (pixelValue == 0) {
                            leftBlackPixelCount++;
                        }
                    }
                }

                for (int row = 0; row < blackCountImg.rows(); row++) {
                    for (int col = leftImageWidth; col < centerImageWidth; col++) {
                        double pixelValue = blackCountImg.get(row, col)[0];
                        if (pixelValue == 0) {
                            centerBlackPixelCount++;
                        }
                    }
                }


                for (int row = 0; row < blackCountImg.rows(); row++) {
                    for (int col = centerImageWidth; col < rightImageWidth; col++) {
                        double pixelValue = blackCountImg.get(row, col)[0];
                        if (pixelValue == 0) {
                            rightBlackPixelCount++;
                        }
                    }
                }*/

                for (int row = 0; row < blackCountImg.rows(); row++) {
                    for (int col = 0; col < blackCountImg.cols(); col++) {
                        double pixelValue = blackCountImg.get(row, col)[0];

                        if(pixelValue == 0) {
                            // left Image
                            if (col < leftImageWidth) {
                                leftBlackPixelCount++;
                            } else if (col >= leftImageWidth && col < centerImageWidth) {
                                centerBlackPixelCount++;
                            } else if (col >= centerImageWidth) {
                                rightBlackPixelCount++;
                            }
                        }
                    }
                }

                if (leftBlackPixelCount > centerBlackPixelCount && leftBlackPixelCount > rightBlackPixelCount) {
                    direction = SpikeMarkerLocation.LEFT;
                } else if (rightBlackPixelCount > leftBlackPixelCount && rightBlackPixelCount > centerBlackPixelCount) {
                    direction = SpikeMarkerLocation.RIGHT;
                } else {
                    direction = SpikeMarkerLocation.CENTER;
                }
                frameCount = 1;

                long time = System.currentTimeMillis();

                endElapsedTime = ""+(time - endTime);
                return blackCountImg;
            } else if (frameCount != 4) {
                frameCount++;
            }
            return frame;
        }

        public SpikeMarkerLocation getDirection() {
            return direction;
        }

        public String getEndTime(){
            return endElapsedTime;
        }
        public String getElapsedTime(){
            return elapsedTime;
        }


    }
}
package org.firstinspires.ftc.teamcode.modules.subsystems;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.modules.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.modules.util.SpikeMarkerLocation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class Vision implements Subsystem {
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal portal;
    private final Vector2d cameraOffset = new Vector2d(5.3, 0);

    public Vision(HardwareMap hardwareMap) {

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setTagLibrary(getCenterStageTagLibrary()) // use tweaked for less error
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setAutoStopLiveView(true)
                .build();
    }

    public List<AprilTagDetection> getDetections() {
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING)
            return new ArrayList<>();

        return aprilTagProcessor.getDetections();
    }

    /**
     * @param botheading In Radians.
     * @return FC Pose of bot.
     */
    public Vector2d getFCPosition(AprilTagDetection detection, double botheading) {
        // get coordinates of the robot in RC coordinates
        // ensure offsets are RC
        double x = detection.ftcPose.x - cameraOffset.getX();
        double y = detection.ftcPose.y - cameraOffset.getY();

        // invert heading to correct properly
        botheading = -botheading;

        // rotate RC coordinates to be field-centric
        double x2 = x * Math.cos(botheading) + y * Math.sin(botheading);
        double y2 = x * -Math.sin(botheading) + y * Math.cos(botheading);
        double absX;
        double absY;

        // add FC coordinates to apriltag position
        VectorF tagpose = detection.metadata.fieldPosition;
        if (detection.metadata.id <= 6) { // first 6 are backdrop tags
            absX = tagpose.get(0) + y2;
            absY = tagpose.get(1) - x2;

        } else { // then just reverse positions
            absX = tagpose.get(0) - y2;
            absY = tagpose.get(1) + x2;
        }
        // Don't send over a pose, as apriltag heading can be off (see discord)
        return new Vector2d(absX, absY);
    }

    public void shutdown() {
        if (portal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_CLOSED)
            return;

        portal.close();
    }

    public static AprilTagLibrary getCenterStageTagLibrary() { // updated custom field coords from michael
        return new AprilTagLibrary.Builder()
                .addTag(1, "BlueAllianceLeft",
                        2, new VectorF(61.75f, 41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(2, "BlueAllianceCenter",
                        2, new VectorF(61.75f, 35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(3, "BlueAllianceRight",
                        2, new VectorF(61.75f, 29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(4, "RedAllianceLeft",
                        2, new VectorF(61.75f, -29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(5, "RedAllianceCenter",
                        2, new VectorF(61.75f, -35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(6, "RedAllianceRight",
                        2, new VectorF(61.75f, -41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(7, "RedAudienceWallLarge",
                        5, new VectorF(-70.25f, -40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(8, "RedAudienceWallSmall",
                        2, new VectorF(-70.25f, -35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(9, "BlueAudienceWallSmall",
                        2, new VectorF(-70.25f, 35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(10, "BlueAudienceWallLarge",
                        5, new VectorF(-70.25f, 40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .build();
    }

    @Override
    public void update() {

    }

    public static class TeamPropPipeline extends OpenCvPipeline {

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


package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.QuaternionBasedImuHelper;
import com.sun.tools.javac.util.ArrayUtils;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealVector;
import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.ColumnMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Vector;



public class ConstructerTags extends LinearOpMode {
    static final String cameraname = "Webcam 1";
    static final int width = 640;
    static final int height = 480;
    static double x = 0;
    static double y = 0;


    public void getDistance() {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, cameraname))
                .setCameraResolution(new Size(width, height))
                .build();


        visionPortal.setProcessorEnabled(tagProcessor,true);
        while (!isStopRequested() && opModeIsActive()) {

            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);


                while (tag.ftcPose.x != 0) {

                    VectorF tagTranslation = tag.metadata.fieldPosition;
                    MatrixF tagRotation = tag.metadata.fieldOrientation.toMatrix();

                    VectorF cameraToTagTranslation = new VectorF(
                            (float) tag.rawPose.x,
                            (float) tag.rawPose.y,
                            (float) tag.rawPose.z
                    );

                    MatrixF cameraToTagRotation = tag.rawPose.R;

                    VectorF zeroVector = new VectorF(0,0,0);
                    MatrixF zeroMatrix = new GeneralMatrixF(3,3);

                    MatrixF tagToCameraRotation = zeroMatrix.subtracted(cameraToTagRotation);
                    VectorF tagToCameraTranslation = tagToCameraRotation.multiplied(zeroVector.subtracted(cameraToTagTranslation));

                    VectorF cameraTranslation = tagTranslation.added(tagRotation.multiplied(tagToCameraTranslation));
                    MatrixF cameraRotation = tagRotation.multiplied(tagToCameraRotation);

                    VectorF robotToCameraTranslation = new VectorF(0,0,0);
                    MatrixF robotToCameraRotation = new GeneralMatrixF(3,3);

                    MatrixF cameraToRobotRotation = zeroMatrix.subtracted(robotToCameraRotation);
                    VectorF cameraToRobotTranslation = cameraToRobotRotation.multiplied(zeroVector.subtracted(robotToCameraTranslation));

                    VectorF robotTranslation = cameraTranslation.added(cameraRotation.multiplied(cameraToRobotTranslation));
                    MatrixF robotRotation = cameraRotation.multiplied(cameraToRobotRotation);

                    Pose2d pos = new Pose2d(
                            robotTranslation.get(0),
                            robotTranslation.get(1),
                            Math.atan2(-robotRotation.get(0,2), robotRotation.get(2,2))
                    );


                }

            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public static void main(String[] args) {


        VectorF tagTranslation = new VectorF(
                (float) 7,
                (float) 2,
                (float) 0
        );
        MatrixF tagRotation = new GeneralMatrixF(3,3, new float[] {
                1f,
                4f,
                5f,
                -9f,
                3f,
                54f,
                52345f,
                336f,
                8f
        });

        VectorF cameraToTagTranslation = new VectorF(
                (float) 100,
                (float) -12,
                (float) 55
        );
        MatrixF cameraToTagRotation = new GeneralMatrixF(3,3, new float[] {
                1f,
                4f,
                5f,
                -9f,
                3f,
                54f,
                52345f,
                336f,
                8f
        });

        VectorF zeroVector = new VectorF(0,0,0);
        MatrixF zeroMatrix = new GeneralMatrixF(3,3);

        MatrixF tagToCameraRotation = zeroMatrix.subtracted(cameraToTagRotation);
        VectorF tagToCameraTranslation = tagToCameraRotation.multiplied(zeroVector.subtracted(cameraToTagTranslation));

        VectorF cameraTranslation = tagTranslation.added(tagRotation.multiplied(tagToCameraTranslation));
        MatrixF cameraRotation = tagRotation.multiplied(tagToCameraRotation);

        VectorF robotToCameraTranslation = new VectorF(0,0,0);
        MatrixF robotToCameraRotation = new GeneralMatrixF(3,3);

        System.out.println(cameraTranslation);
    }
}



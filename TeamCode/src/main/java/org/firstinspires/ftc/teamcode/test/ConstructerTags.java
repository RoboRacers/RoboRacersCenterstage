package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.QuaternionBasedImuHelper;
import com.sun.tools.javac.util.ArrayUtils;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealVector;
import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Vector;


public class ConstructerTags extends LinearOpMode {
    static final String cameraname = "Webcam 1";
    static final int width = 640;
    static final int height = 360;
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

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);


                while (tag.ftcPose.x != 0) {


                    VectorF Prel = new VectorF(
                            (float) tag.ftcPose.x,
                            (float) tag.ftcPose.y,
                            (float) tag.ftcPose.z
                    );

                    VectorF P1 = tag.metadata.fieldPosition;

                    MatrixF Rrel = tag.rawPose.R;

                    MatrixF R1 = tag.metadata.fieldOrientation.toMatrix();

                    MatrixF Rnew = Rrel.multiplied(R1);

                    VectorF Pfinal = P1.added(Rnew.multiplied(Prel));

                    telemetry.addData("Position", Pfinal);


                }

            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public static void main(String[] args) {
        VectorF Prel = new VectorF(
                (float) -10,
                (float) 3,
                (float) 6
        );

        VectorF P1 = AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(1).fieldPosition;

        MatrixF Rrel = AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(2).fieldOrientation.toMatrix();

        MatrixF R1 = AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(1).fieldOrientation.toMatrix();

        MatrixF Rnew = Rrel.multiplied(R1);

        VectorF Pfinal = P1.added(Rnew.multiplied(Prel));

        System.out.println(Pfinal);
    }
}



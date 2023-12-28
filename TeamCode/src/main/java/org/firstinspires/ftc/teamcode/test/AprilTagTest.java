package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.modules.subsystems.Vision;
import org.firstinspires.ftc.teamcode.modules.vision.Transform3d;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.ArrayList;

@TeleOp(name = "April Tag Detection Test", group = "Test")
public class AprilTagTest extends LinearOpMode {

    public Vision vision;

    @Override
    public void runOpMode() throws InterruptedException {

        vision = new Vision(hardwareMap);

        waitForStart();


        while (!isStopRequested()) {

            if (gamepad1.dpad_down) {
                vision.visionPortal.stopStreaming();

            } else if (gamepad1.dpad_up) {
                vision.visionPortal.resumeStreaming();
            } else if (gamepad1.dpad_left) {
                vision.tagProcessor.getDetections().get(0).metadata.fieldOrientation.toMatrix();
            } else if (gamepad1.dpad_right) {
                AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(1).fieldOrientation.toMatrix();
            }
            telemetry.addData("Portal State", vision.visionPortal.getCameraState());
            telemetry.addData("Detections", vision.tagProcessor.getDetections());

            ArrayList<AprilTagDetection> tags = vision.tagProcessor.getDetections();

            ArrayList<Pose2d> poses = new ArrayList<>();

            for (AprilTagDetection tag: tags) {
                if (tag.metadata != null) {


                    Transform3d tagPose = new Transform3d(
                            tag.metadata.fieldPosition,
                            tag.metadata.fieldOrientation.toMatrix()
                    );



                    Transform3d cameraToTagTransform = new Transform3d(
                            new VectorF(
                                    (float) tag.rawPose.x,
                                    (float) tag.rawPose.y,
                                    (float) tag.rawPose.z
                            ),
                            tag.rawPose.R
                    );


                    Transform3d tagToCameraTransform = cameraToTagTransform.unaryMinusInverse();

                    Transform3d cameraPose = tagPose.plus(tagToCameraTransform);

                    Transform3d robotToCameraTransform = new Transform3d();
                    Transform3d cameraToRobotTransform = robotToCameraTransform.unaryMinusInverse();

                    Transform3d robotPose = cameraPose.plus(cameraToRobotTransform);

                    poses.add(robotPose.toPose2d());


                }
            }

            telemetry.addData("Possible Pose", poses);
            telemetry.update();

        }
        vision.visionPortal.close();

    }

    public static void main(String[] args) {
        MatrixF R = new Quaternion().toMatrix();

        System.out.println(R);
    }

}



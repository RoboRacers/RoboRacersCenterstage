package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.modules.drive.ThreeTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.modules.statemachines.SlidesSM;
import org.firstinspires.ftc.teamcode.modules.subsystems.Vision;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.modules.util.SpikeMarkerLocation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

// Localization is doesn't show drift, follower if it does

@Config
@Autonomous(name = "Blue Far Side Auton Raw", group = "16481-Centerstage-Raw")
public class BlueFarAutonRaw extends LinearOpMode{

    RobotCore robot;

    Vision.TeamPropPipeline teamPropDetectionPipeline = null;

    SpikeMarkerLocation spikeMarkerLocation = SpikeMarkerLocation.CENTER; // Defaults to center

    TrajectorySequence LeftSpikeMarker;
    TrajectorySequence LeftDeposit1;
    TrajectorySequence CenterSpikeMarker;
    TrajectorySequence CenterDeposit1;
    TrajectorySequence CenterDeposit2;
    TrajectorySequence CenterCycle;
    TrajectorySequence RightSpikeMarker;
    TrajectorySequence RightDeposit1;

    public static double backBoardX = 48.66;

    public static double wait = 0.1;



    @Override
    public void runOpMode() {

        OpenCvCamera camera;

        robot = new RobotCore(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap
                .get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                teamPropDetectionPipeline = new Vision.TeamPropPipeline();
                camera.setPipeline(teamPropDetectionPipeline);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        Pose2d startLocation = new Pose2d(-43, 62.00, Math.toRadians(-90));
        robot.drive.setPoseEstimate(startLocation);

        if (isStopRequested()) return;



        CenterSpikeMarker = robot.drive.trajectorySequenceBuilder(startLocation)
                .waitSeconds(wait)
                .addDisplacementMarker(() -> {
                })
                .splineToConstantHeading(new Vector2d(-40.11, 30.75), Math.toRadians(270.00))
                //.splineToConstantHeading(new Vector2d(-40.5, -40.16), Math.toRadians(-270.00))
                //move to starter stack
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .splineTo(new Vector2d(-62.00, 40.53), Math.toRadians(180))
                // Sweep the stack
                .splineToConstantHeading(new Vector2d(-62.58, 31.53), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-62.58, 35.53), Math.toRadians(180))
                // wait to intake
                .waitSeconds(0.75)
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {

                })
                .splineToConstantHeading(new Vector2d(-58.30, 31.53), Math.toRadians(270))
                // wait and them outtake
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {

                })
                .splineToConstantHeading(new Vector2d(-58.30, 15.53), Math.toRadians(270))

                //move to backdrop

                .splineToConstantHeading(new Vector2d(-43.43, 8.13), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-20.16, 8.7), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(38.41, 8.73), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(3, () -> {

                })
                .build();

        CenterDeposit1 = robot.drive.trajectorySequenceBuilder(CenterSpikeMarker.end())
                .splineToConstantHeading(new Vector2d(backBoardX, 33.7), Math.toRadians(0))//in front of backdrop
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {


                })
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                })
                .splineToConstantHeading(new Vector2d(backBoardX-3, 32.55), Math.toRadians(0))//in front of backdrop
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {

                })
                .UNSTABLE_addTemporalMarkerOffset(0.33, () -> {
                    //robot.drive.followTrajectorySequenceAsync(CenterCycle);
                })
                .build();


        boolean manualPropControl = true;

        while(!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("SELF CHECK -----");

            // Checks if the positions of the encoders to make sure they are not unplugged
            robot.drive.updatePoseEstimate();
            ThreeTrackingWheelLocalizer localizer = (ThreeTrackingWheelLocalizer) robot.drive.getLocalizer();
            List<Double> deadwheelPositions = localizer.getWheelPositions();

            telemetry.addData("Left Encoder Pos", deadwheelPositions.get(0));
            telemetry.addData("Right Encoder Pos", deadwheelPositions.get(1));
            telemetry.addData("Perpendicular Encoder Pos", deadwheelPositions.get(2));

            if (deadwheelPositions.get(0) == 0) {
                telemetry.addLine("LEFT ENCODER UNPLUGGED, Check wiring of Port x");
            }
            if (deadwheelPositions.get(1) == 0) {
                telemetry.addLine("RIGHT ENCODER UNPLUGGED, Check wiring of Port x");
            }
            if (deadwheelPositions.get(2) == 0) {
                telemetry.addLine("PERPENDICULAR ENCODER UNPLUGGED, Check wiring of Port x");
            }


            // Vision code here
            telemetry.addLine("VISION -----");

            // Switch between manual and automatic vision control
            if (gamepad1.left_bumper) {
                manualPropControl = true;
            } else if (gamepad1.right_bumper) {
                manualPropControl = false;
            }

            if (!manualPropControl) {
                telemetry.addLine("Prop Detection mode is AUTOMATIC");
                if (teamPropDetectionPipeline != null) {
                    spikeMarkerLocation = teamPropDetectionPipeline.getDirection();
                    telemetry.addData("Spike Marker Location", spikeMarkerLocation);
                } else {
                    telemetry.addLine("Camera not initialized");
                }
            } else {
                telemetry.addLine("Prop Detection is MANUAL");
                if (gamepad1.square) {
                    spikeMarkerLocation = SpikeMarkerLocation.LEFT;
                } else if (gamepad1.circle) {
                    spikeMarkerLocation = SpikeMarkerLocation.CENTER;
                } else if (gamepad1.triangle) {
                    spikeMarkerLocation = SpikeMarkerLocation.RIGHT;
                }
                telemetry.addData("Spike Marker Location", spikeMarkerLocation);
            }

            telemetry.update();
        }

        camera.stopStreaming();

        waitForStart();

        if (isStopRequested()) return;

        // Runs the trajectory based on the start location
        switch (spikeMarkerLocation) {
            // Reversed
            case CENTER:
                robot.drive.followTrajectorySequenceAsync(CenterSpikeMarker);
                break;
            default:
                break;
        }

        while (opModeIsActive() && !isStopRequested()) {
            robot.drive.update();


            telemetry.update();
        }

    }

}

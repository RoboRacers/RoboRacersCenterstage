package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.opencv.SignalDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

// Localization is doesn't show drift, follower if it does

@Config
@Autonomous(name = "Demo Autonomous", group = "16481-Power-Play")
public class AutoopDemo extends LinearOpMode {

    // Declare the motors and servos
    DcMotorEx motorLeft;
    DcMotorEx motorRight;
    Servo claw;

    public enum STATE_POSITION {
        STATE_POSITION_SP9,
        STATE_POSITION_SP0,
        STATE_POSITION_SP1,
        STATE_POSITION_SP2,
        STATE_POSITION_SP3
    }

    //Setting Current state for each section to desired starting state
    public STATE_POSITION RobotPosition = STATE_POSITION.STATE_POSITION_SP9;

    @Override
    public void runOpMode() {
        /* Telemetry */
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        /* Motor and Servo setup */
        motorLeft = hardwareMap.get(DcMotorEx.class, "LiftLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "LiftRight");
        claw = hardwareMap.get(Servo.class, "claw");

        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /* April Tag detection Code */
        OpenCvCamera camera;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.
                get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        SignalDetection mySignalDetection = new SignalDetection(camera, (MultipleTelemetry) telemetry);
        mySignalDetection.openConnection();

        sleep(100);
        int tagID=-1;

        telemetry.addData("# Detecting AprilTag ","");
        telemetry.update();

        while(!isStopRequested() && !opModeIsActive()) {
            // Repeatedly update the signal ID with new detections from init to opmode start
            tagID = mySignalDetection.CheckSignal();
            telemetry.addData("# Tag ID: ", tagID);
            telemetry.addData("Position selected ", RobotPosition);
            telemetry.update();
        }

        camera.closeCameraDevice();

        // Import Roadrunner Trajectories
        RoadrunnerPointDataset Trajectories = new RoadrunnerPointDataset(drive, (MultipleTelemetry) telemetry, motorRight, motorLeft, claw);

        claw.setPosition(0.45);
        waitForStart();

        if (isStopRequested()) return;

        Boolean cycle = false;
        Boolean test = false;
        while(opModeIsActive()){
            /* Running Trajectories */
            if(cycle == false && test == false){
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
                if (tagID == 0) { Trajectories.DemoAuto1(); }
                else if (tagID == 1) { Trajectories.DemoAuto2(); }
                else if (tagID == 2) { Trajectories.DemoAuto3(); }
                break;
            }

            telemetry.addData("Autonomous Finished", "");
            telemetry.update();
        }
    }
}

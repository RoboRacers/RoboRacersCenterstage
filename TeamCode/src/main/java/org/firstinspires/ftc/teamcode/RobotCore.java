package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.opencv.SignalDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

/**
 * This class contains the declaration and setup for all motors and cameras on the robot.
 * To use this class in an opmode, declare:
 * RobotCore robot = new RobotCore(hardwareMap);
 * You can call motors and such through:
 * robot.componentName.command();
 */

public class RobotCore {

    // Declare the motors and servos
    public DcMotorEx motorLeft;
    public DcMotorEx motorRight;
    public Servo claw;
    public SampleMecanumDrive drive;
    public OpenCvCamera camera;

    public Gamepad gamepad1;
    public Gamepad gamepad2;

    public HardwareMap hardwareMap;

    // Constructor, run during initialization
    public RobotCore(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2){

        // Telemetry object commented out because we don't need it
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        this.hardwareMap = hardwareMap;
    }

    public void initOpenCV (){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.
                get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        SignalDetection mySignalDetection = new SignalDetection(camera, (MultipleTelemetry) telemetry);
        mySignalDetection.openConnection();
    }

    // Function to set the arm position
    public void setArmPos(int pos, double liftSpeed) {
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorRight.setTargetPosition(pos);
        motorLeft.setTargetPosition(pos);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setPower(liftSpeed);
        motorRight.setPower(liftSpeed);
    }

    // Function to set the claw position
    public void setClawPos(double posclaw) {
        claw.setPosition(posclaw);
        gamepad1.rumble(500);
        gamepad2.rumble(500);
    }

}

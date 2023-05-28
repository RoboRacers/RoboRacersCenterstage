package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.modules.opencv.ConeDetection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Disabled
@Config
@TeleOp(name = "TeleOp State Machines", group = "16481-Power-Play")
public class TeleopStateMachines extends LinearOpMode {

    public enum STATE_CLAW {
        STATE_CLAW_OPEN,                // claw opens
        STATE_CLAW_CLOSED              //claw closes
    }

    //cone detect < 150MM
    public enum STATE_CLAW_RANGE {
        STATE_CONE_DETECT,
        STATE_CONE_NOT_DETECTED
    }
    /*ToDo : Range sensor for arm height detection
    In auto, when picking up starter stack, move arm down until cone detection happens,
    then pick up cone, cycle, and move arm back down to the same position. Repeat process to cycle all the cones
    */

    public enum STATE_ARM {
        STATE_ARM_LEVEL_0,               //raise arm height to medium pole
        STATE_ARM_LEVEL_1,               //raise arm height to low pole
        STATE_ARM_LEVEL_2,              //raise arm height to high pole
        STATE_ARM_LEVEL_3,              //raise arm height to pick up cone
        STATE_ARM_MANUAL,                 //manual lower arm down
    }
    /*
    public enum STATE_DRIVE{
        STATE_DRIVE_STOP,                    //drives stop
        STATE_DRIVE_FORWARD,                 //drives forward
        STATE_DRIVE_BACKWARD,                //drives backward
        STATE_DRIVE_STRAFE_LEFT,              //strafe left
        STATE_DRIVE_STRAFE_RIGHT,             //strafe right
        STATE_DRIVE_TURN_LEFT,                //rotate left
        STATE_DRIVE_TURN_RIGHT               //rotate right
    }
    public enum STATE_ROADRUNNER {
        //ToDo: Classify each preset
        STATE_ROADRUNNER_POS0,
        STATE_ROADRUNNER_POS1,          //Different roadrunner presets that are used in game
        STATE_ROADRUNNER_POS2,
        STATE_ROADRUNNER_POS3,
        STATE_ROADRUNNER_POS4,
        STATE_ROADRUNNER_POS5,
        STATE_ROADRUNNER_POS6,
        STATE_ROADRUNNER_POS7,
        STATE_ROADRUNNER_POS8,
        STATE_ROADRUNNER_POS9,
        STATE_ROADRUNNER_POS10,
        STATE_ROADRUNNER_POS11,
        STATE_ROADRUNNER_POS12,
        STATE_ROADRUNNER_POS13,
        STATE_ROADRUNNER_POS14,
        STATE_ROADRUNNER_POS15
    }
    */
    /**
     * Lift Level constants
     */
    final int LIFT_LEVEL_0 = 0;
    final int LIFT_LEVEL_1 = -600;
    final int LIFT_LEVEL_2 = -900;
    final int LIFT_LEVEL_3 = -1200;

    final double CLAW_OPEN = 0;
    final double CLAW_CLOSE = 0.7;

    /**
     * Setting Current state for each section to desired starting state
     * these states come from end of autocode. Check with autocode where it ends
     */
    // public STATE_DRIVE DriveState = STATE_DRIVE.STATE_DRIVE_STOP;
    // public STATE_ROADRUNNER RoadrunnerState = STATE_ROADRUNNER.STATE_ROADRUNNER_POS0;
    public STATE_CLAW ClawState = STATE_CLAW.STATE_CLAW_OPEN;
    public STATE_ARM ArmState = STATE_ARM.STATE_ARM_LEVEL_0;
    public STATE_CLAW_RANGE ClawRangeState = STATE_CLAW_RANGE.STATE_CONE_NOT_DETECTED;


    /**
     * Timer to increment servo. Servo increment to next position when timer reach a set value
     */
   // ElapsedTime timer_1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private DcMotorEx LiftMotorLeft;
    private DcMotorEx LiftMotorRight;
    private Servo clawServo;
    private DistanceSensor clawRange;


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        /** **********************************************************************
         * Roadrunner Initialisation
         **********************************************************************/
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /** **********************************************************************
         * cone detection Code
         **********************************************************************/
        /*OpenCvCamera camera;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.
                get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        ConeDetection myConeDetection = new ConeDetection(camera);
        */
        /** **********************************************************************
         * Claw Initialisation
         **********************************************************************/
        clawServo = hardwareMap.get(Servo.class, "claw");
        clawRange = hardwareMap.get(DistanceSensor.class, "clawRange");

        /** **********************************************************************
         * Lift Initialisation
         **********************************************************************/
        LiftMotorLeft  = hardwareMap.get(DcMotorEx.class, "LiftLeft");
        LiftMotorRight  = hardwareMap.get(DcMotorEx.class, "LiftRight");

        LiftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        LiftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LiftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LiftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()){
            /** **********************************************************************
             * Drive Code
             **********************************************************************/
            drive.setWeightedDrivePower(
                 new Pose2d(
                     -gamepad1.left_stick_y*.80,
                     -gamepad1.left_stick_x*.80, //imperfect strafing fix, must be tuned for new drivetrain
                     -gamepad1.right_stick_x*.75
                 )
            );
            drive.update();

            /** **********************************************************************
             * claw range cade
             **********************************************************************/
            switch (ClawRangeState) {
                case STATE_CONE_DETECT:
                    telemetry.addLine("cone detected");
                    ClawState = STATE_CLAW.STATE_CLAW_CLOSED;
                    break;
                case STATE_CONE_NOT_DETECTED:
                    telemetry.addLine("Cone not detected");
                    telemetry.addData("Distance:", clawRange.getDistance(DistanceUnit.MM));
                    if(clawRange.getDistance(DistanceUnit.MM) < 150) {
                        ClawRangeState = STATE_CLAW_RANGE.STATE_CONE_DETECT;
                    }
                    break;
            }
            /** **********************************************************************
             *  claw code
             **********************************************************************/
            switch (ClawState) {
                case STATE_CLAW_OPEN:
                    telemetry.addLine("Claw open");
                    clawServo.setPosition(CLAW_OPEN);
                    if(gamepad2.left_bumper) {
                        ClawState = STATE_CLAW.STATE_CLAW_CLOSED;
                        gamepad1.rumble(500);
                        gamepad2.rumble(500);
                    }
                    break;
                case STATE_CLAW_CLOSED:
                    telemetry.addLine("Claw close");
                    clawServo.setPosition(CLAW_CLOSE);
                    if(gamepad2.right_bumper) {
                        ClawState = STATE_CLAW.STATE_CLAW_OPEN;
                        ClawRangeState = STATE_CLAW_RANGE.STATE_CONE_NOT_DETECTED;
                        gamepad1.rumble(500);
                        gamepad2.rumble(500);
                    }
                    break;
            }
            /** **********************************************************************
             * lift code
             **********************************************************************/
            switch (ArmState) {
                case STATE_ARM_LEVEL_0:
                    setArmPosition(LIFT_LEVEL_0);
                    telemetry.addLine("Level 0");
                    if(gamepad2.dpad_left)
                        ArmState = STATE_ARM.STATE_ARM_LEVEL_1;
                    if(gamepad2.dpad_right)
                        ArmState = STATE_ARM.STATE_ARM_LEVEL_2;
                    if(gamepad2.dpad_up)
                        ArmState = STATE_ARM.STATE_ARM_LEVEL_3;
                    if(gamepad2.a)
                        ArmState = STATE_ARM.STATE_ARM_MANUAL;
                    break;
                case STATE_ARM_LEVEL_1:
                    telemetry.addLine("Level 1");
                    setArmPosition(LIFT_LEVEL_1);
                    if(gamepad2.dpad_down)
                        ArmState = STATE_ARM.STATE_ARM_LEVEL_0;
                    if(gamepad2.dpad_right)
                        ArmState = STATE_ARM.STATE_ARM_LEVEL_2;
                    if(gamepad2.dpad_up)
                        ArmState = STATE_ARM.STATE_ARM_LEVEL_3;
                    break;
                case STATE_ARM_LEVEL_2:
                    telemetry.addLine("Level 2");
                    setArmPosition(LIFT_LEVEL_2);
                    if(gamepad2.dpad_down)
                        ArmState = STATE_ARM.STATE_ARM_LEVEL_0;
                    if(gamepad2.dpad_left)
                        ArmState = STATE_ARM.STATE_ARM_LEVEL_1;
                    if(gamepad2.dpad_up)
                        ArmState = STATE_ARM.STATE_ARM_LEVEL_3;
                    if(gamepad2.a)
                        ArmState = STATE_ARM.STATE_ARM_MANUAL;
                    break;
                case STATE_ARM_LEVEL_3:
                    telemetry.addLine("Level 3");
                    setArmPosition(LIFT_LEVEL_3);
                    if(gamepad2.dpad_down)
                        ArmState = STATE_ARM.STATE_ARM_LEVEL_0;
                    if(gamepad2.dpad_left)
                        ArmState = STATE_ARM.STATE_ARM_LEVEL_1;
                    if(gamepad2.dpad_right)
                        ArmState = STATE_ARM.STATE_ARM_LEVEL_2;
                    if(gamepad2.a)
                        ArmState = STATE_ARM.STATE_ARM_MANUAL;
                    break;
                    /*
                case STATE_ARM_MANUAL:
                    telemetry.addData("Right sticky y", gamepad2.right_stick_y);
                    moveArm();
                    if(gamepad2.dpad_down)
                        ArmState = STATE_ARM.STATE_ARM_LEVEL_0;
                    if(gamepad2.dpad_left)
                        ArmState = STATE_ARM.STATE_ARM_LEVEL_1;
                    if(gamepad2.dpad_right)
                        ArmState = STATE_ARM.STATE_ARM_LEVEL_2;
                    if(gamepad2.b)
                        ArmState = STATE_ARM.STATE_ARM_LEVEL_3;
                    break;
                     */
            }
            telemetry.update();
        }
    }

    public void setArmPosition(int target)
    {
        if (LiftMotorLeft.isBusy() && LiftMotorRight.isBusy()) {
            telemetry.addLine("returning from busy");
            telemetry.update();
            return;
        }

        telemetry.addData("Moving arm to target: ", target);
        telemetry.update();

        LiftMotorLeft.setPower(0);
        LiftMotorRight.setPower(0);

        LiftMotorRight.setTargetPosition(target);
        LiftMotorLeft.setTargetPosition(target);

        LiftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LiftMotorLeft.setPower(.5);
        LiftMotorRight.setPower(.5);
    }

    public void moveArm()
    {
        if (LiftMotorLeft.isBusy() || LiftMotorRight.isBusy())
            return;

        telemetry.addLine("Moving arm in manual mode");
        LiftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LiftMotorLeft.setPower(gamepad2.right_stick_y);
        LiftMotorRight.setPower(gamepad2.right_stick_y);
    }
}

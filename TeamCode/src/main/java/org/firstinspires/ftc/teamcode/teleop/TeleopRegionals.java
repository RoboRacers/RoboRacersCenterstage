package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.test.LiftPID;

@TeleOp(name = "Teleop For Regionals", group = "16481-Power-Play")
public class TeleopRegionals extends LinearOpMode {

    DcMotorEx motorLeft;
    DcMotorEx motorRight;

    DistanceSensor armRangeSensor;
    DistanceSensor clawRangeSensor;

    Servo claw;

    // Drive Control Constants
    final double driveSensitivity = 0.80;
    final double turnSensitivity = 0.75;

    // Lift and Servo Constants
    final int liftLow = 0;
    final int liftHigherThanLow = -750;
    final int liftMid = -1075;
    final int liftHigh = -1350;



    double liftSpeed = 0.4;

    final double closed = 0.45;
    final double open = 0;

    // Sensor Control Values
    double targetPos = liftLow;
    double currentArmPos;
    double clawDist;

    int motorEncoderAvg;

    // Lift Control States
    boolean RunToTarget = false;
    boolean EncoderPIDRunning = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Drive Setup
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Motor and Servo Setup
        motorLeft = hardwareMap.get(DcMotorEx.class, "LiftLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "LiftRight");
        claw = hardwareMap.get(Servo.class, "claw");

        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Sensor Setup
        armRangeSensor = hardwareMap.get(DistanceSensor.class, "armRange");
        clawRangeSensor = hardwareMap.get(DistanceSensor.class, "clawRange");

        // PID Setup
        LiftPID myLift = new LiftPID();

        while (opModeInInit()) {
            currentArmPos = armRangeSensor.getDistance(DistanceUnit.MM);
        }

        while (!isStopRequested()) {

            /*--Drive Control--*/
            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y*driveSensitivity, -gamepad1.left_stick_x*driveSensitivity, -gamepad1.right_stick_x*turnSensitivity));
            drive.update();

            /*--Arm Control--*/
            currentArmPos = armRangeSensor.getDistance(DistanceUnit.MM);
            clawDist = clawRangeSensor.getDistance(DistanceUnit.MM);

            // Arm manual control
            // If Stick Goes up, Go up
            if (gamepad2.left_stick_y < -0.1) {
                ResetMotorMode();
                RunToTarget = false;
                EncoderPIDRunning = false;

                if (gamepad2.left_stick_y < -0.5){
                    setMotorSpeed(-0.25);
                } else {
                    setMotorSpeed(gamepad2.left_stick_y/2);
                }
            }
            // If Stick goes down, go down
            else if (gamepad2.left_stick_y > 0.1) {
                ResetMotorMode();
                RunToTarget = false;
                EncoderPIDRunning = false;
                if (gamepad2.left_stick_y > 0.5){
                    setMotorSpeed(0.25/2);
                } else {
                    setMotorSpeed(gamepad2.left_stick_y/4);
                }
            }
            // Otherwise, lock position
            else if (gamepad2.left_stick_y < 0.5
                    && -0.5 < gamepad2.left_stick_y
                    && RunToTarget == false && EncoderPIDRunning == false){
                motorEncoderAvg = (motorLeft.getCurrentPosition() + motorRight.getCurrentPosition())/2;
                ArmPosition(motorEncoderAvg);
                RunToTarget = false;
                EncoderPIDRunning = true;
            }

            // Arm Preset Control
            if(gamepad2.dpad_up) {
                // Set arm Position to High
                ArmPosition(liftHigh);
            } else if(gamepad2.dpad_down) {
                // Set arm Position to Low
                ArmPosition(liftLow);
            }else if(gamepad2.dpad_left) {
                // Set arm Position to Medium
                ArmPosition(liftMid);
            }else if(gamepad2.dpad_right) {
                // Set arm Position to a bit lower than High
                ArmPosition(liftHigherThanLow);
            }

            // Update the Lift Motors based on the TargetPos and CurrentArmPos
            if (RunToTarget) {
                if (currentArmPos > targetPos + 10) {
                    setMotorSpeed(-myLift.getTargetVelocity( targetPos - currentArmPos));
                } else if (currentArmPos < targetPos - 10){
                    setMotorSpeed(-myLift.getTargetVelocity( targetPos - currentArmPos));
                }else if (targetPos - 5 < currentArmPos && currentArmPos < targetPos + 5) {
                    motorEncoderAvg = (motorLeft.getCurrentPosition() + motorRight.getCurrentPosition())/2;
                    ArmPosition(motorEncoderAvg);
                    RunToTarget = false;
                    EncoderPIDRunning = true;
                }
            }

            /*--Claw Control--*/
            if(gamepad2.right_bumper) {
                claw(closed);
            } else if(gamepad2.left_bumper){
                claw(open);
            }
            if (gamepad2.right_trigger > 0.5 && clawDist < 35) {
                claw(closed);
            }


            // Telemetry
            telemetry.addData("Gamepad 2 Left Stick X", gamepad2.left_stick_y);
            telemetry.addData("Left Motor", motorLeft.getPower());
            telemetry.addData("Right Motor", motorRight.getPower());
            telemetry.addData("Using Lift Controller", RunToTarget);
            telemetry.addData("Using Encoder PID", EncoderPIDRunning);
            telemetry.addData("Target Position", targetPos);
            telemetry.addData("Current Arm Postion", String.format("%.01f mm", currentArmPos));
            telemetry.addData("Current Claw Distance", String.format("%.01f mm", clawDist));
            telemetry.update();
        }
    }

    public void ArmPosition(int pos) {
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorRight.setTargetPosition(pos);
        motorLeft.setTargetPosition(pos);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setPower(.75);
        motorRight.setPower(.75);
    }

    public void ResetMotorMode() {
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorSpeed(double speed)
    {
        motorLeft.setPower(speed);
        motorRight.setPower(speed);
    }

    public void claw(double posclaw) {
        claw.setPosition(posclaw);
        gamepad1.rumble(500);
        gamepad2.rumble(500);
    }
}

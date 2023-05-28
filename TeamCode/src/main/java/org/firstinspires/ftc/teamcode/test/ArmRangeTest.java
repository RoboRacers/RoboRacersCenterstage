package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ArmRangeTest extends LinearOpMode {

    DcMotorEx motorLeft;
    DcMotorEx motorRight;

    DistanceSensor armRangeSensor;
    DistanceSensor clawRangeSensor;

    Servo claw;

    final double liftLow = 20.0;
    final double liftHigherThanLow = 300.0;
    final double liftMid = 600.0;
    final double liftHigh = 800.0;

    double liftSpeed = 0.4;

    final double closed = 0.25;
    final double open = 0;

    double targetPos = liftLow;
    double currentArmPos;
    double clawDist;

    boolean RunToTarget = false;
    boolean EncoderPIDRunning = false;

    int motorEncoderAvg;


    @Override
    public void runOpMode() throws InterruptedException {
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

        armRangeSensor = hardwareMap.get(DistanceSensor.class, "armRange");
        clawRangeSensor = hardwareMap.get(DistanceSensor.class, "clawRange");

        LiftPID myLift = new LiftPID();

        while (opModeInInit()) {
            currentArmPos = armRangeSensor.getDistance(DistanceUnit.MM);
        }

        while (!isStopRequested()) {
            currentArmPos = armRangeSensor.getDistance(DistanceUnit.MM);
            clawDist = clawRangeSensor.getDistance(DistanceUnit.MM);

            // Arm manual control
            if (gamepad2.left_stick_y < -0.1) {
                ResetMotorMode();
                RunToTarget = false;
                EncoderPIDRunning = false;
                // If Stick Goes Up, Go Up
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
            } else if (gamepad2.left_stick_y < 0.5
                    && -0.5 < gamepad2.left_stick_y
                    && RunToTarget == false && EncoderPIDRunning == false){
                motorEncoderAvg = (motorLeft.getCurrentPosition() + motorRight.getCurrentPosition())/2;
                ArmPosition(motorEncoderAvg);
                RunToTarget = false;
                EncoderPIDRunning = true;
            }

            // Arm level control
            if(gamepad2.dpad_up) {
                ResetMotorMode();
                RunToTarget = true;
                targetPos = 800.0;
            } else if(gamepad2.dpad_down) {
                ResetMotorMode();
                RunToTarget = true;
                targetPos = 20.0;
            } else if(gamepad2.dpad_left) {
                ResetMotorMode();
                RunToTarget = true;
                targetPos = 600.0;
            } else if(gamepad2.dpad_right) {
                ResetMotorMode();
                RunToTarget = true;
                targetPos = 400.0;
            }

            // Claw Control
            if(gamepad2.right_bumper) {
                claw(closed);
            } else if(gamepad2.left_bumper){
                claw(open);
            }
            if (gamepad2.right_trigger > 0.5 && clawDist < 35) {
                claw(closed);
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

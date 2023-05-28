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

@TeleOp(name = "Teleop For League Tournament", group = "16481-Power-Play")
public class TeleopLM2 extends LinearOpMode {

    DcMotorEx motorLeft;
    DcMotorEx motorRight;

    Servo claw;

    DistanceSensor clawRangeSensor;

    double clawDist;

    int previousTargetEncoderValue;
    int targetEncoderValue;
    int previousCommonModifier;
    int commonModifier = 0;
    int baseEncoderValue;
    final int liftLow = 0;
    final int liftHigherThanLow = -750;
    final int liftMid = -1075;
    final int liftHigh = -1350;

    double driveSensitivity = .5;
    double turnSensitivity = .75;
    double liftSpeed = .5;

    int motorEncoderAvg;

    final double closed = 0.25;
    final double open = 0;

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
        //clawRangeSensor = hardwareMap.get(DistanceSensor.class, "clawRange");


        while (opModeInInit()) {
            claw(open);
        }

        while (!isStopRequested()) {
            //clawDist = clawRangeSensor.getDistance(DistanceUnit.MM);
            previousCommonModifier = commonModifier;
            previousTargetEncoderValue = targetEncoderValue;
            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y*driveSensitivity, -gamepad1.left_stick_x*driveSensitivity, -gamepad1.right_stick_x*turnSensitivity));
            drive.update();

            if(gamepad2.right_bumper) {
                claw.setPosition(closed);
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            } else if(gamepad2.left_bumper){
                claw.setPosition(open);
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            } else if(gamepad2.dpad_up) {
                // Set arm Position to High
                baseEncoderValue = liftHigh;
            } else if(gamepad2.dpad_down) {
                // Set arm Position to Low
                baseEncoderValue = liftLow;
            }else if(gamepad2.dpad_left) {
                // Set arm Position to Medium
                baseEncoderValue = liftMid;
            }else if(gamepad2.dpad_right) {
                // Set arm Position to a bit lower than High
                baseEncoderValue = liftHigherThanLow;
            }else if(gamepad2.a) {
                // Change the encoder modifier down
                commonModifier = 100;
                //ArmPosition(motorLeft.getCurrentPosition() + 100);
            }else if(gamepad2.b) {
                // Change the encoder modifier up
                commonModifier = -100;
                //ArmPosition(motorLeft.getCurrentPosition() - 100);
            } else if(gamepad2.x) {
                // Change the encoder modifier up
                commonModifier = 0;
                //ArmPosition(motorLeft.getCurrentPosition() - 100);
            }
            targetEncoderValue = baseEncoderValue + commonModifier;
            /*
            // Arm manual control
            // If Stick Goes up, Go up
            if (gamepad2.left_stick_y < -0.1) {
                ResetMotorMode();
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
                targetEncoderValue = motorEncoderAvg;
                EncoderPIDRunning = true;
            }

             */

            /*--Claw Control--
            if (gamepad2.right_trigger > 0.5 && clawDist < 35) {
                claw(closed);
            }

             */

            if (previousTargetEncoderValue != targetEncoderValue){
                ArmPosition(targetEncoderValue);
            }



            // Telemetry
            telemetry.addData("Roboracers Teleop for League Tournament", "");
            telemetry.addData("Gamepad 2 Left Stick Y", gamepad2.left_stick_y);
            telemetry.addData("Left Motor Power", motorLeft.getPower());
            telemetry.addData("Right Motor Power", motorRight.getPower());
            telemetry.addData("RunToPosition Desired Encoder Value", targetEncoderValue);
            telemetry.addData("Left Motor Encoder Value", motorLeft.getCurrentPosition());
            telemetry.addData("Right Motor Encoder Value", motorRight.getCurrentPosition());
            telemetry.addData("Common Encoder Modifier", commonModifier);
            telemetry.update();

        }
    }

    // Function to set the arm position
    public void ArmPosition(int pos) {
        targetEncoderValue = pos;
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
    public void claw(double posclaw) {
        claw.setPosition(posclaw);
        gamepad1.rumble(500);
        gamepad2.rumble(500);
    }

    // Function to set the claw position
    public void changeCommonModifier(int modifier, int change, int down) {
        if (down == -1){
            modifier = modifier - change;
        } else if (down == 1){
            modifier = modifier + change;
        }
        commonModifier = modifier;
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
}

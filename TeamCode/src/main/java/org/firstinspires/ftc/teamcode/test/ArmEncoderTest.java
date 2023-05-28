package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Arm Encoder Test")
public class ArmEncoderTest extends LinearOpMode {

    DcMotorEx motorLeft;
    DcMotorEx motorRight;

    Servo claw;

    int baseEncoderValue;
    int targetEncoderValue;
    int previousTargetEncoderValue;


    final int liftLow = 0;
    final int liftHigherThanLow = -750;
    final int liftMid = -1075;
    final int liftHigh = -1350;

    final double closed = 0.45;
    final double open = 0;

    double targetPos = liftLow;
    double currentArmPos;

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

        while (opModeInInit()) {

        }

        while (!isStopRequested()) {
            previousTargetEncoderValue = targetEncoderValue;
            motorEncoderAvg = (motorLeft.getCurrentPosition()+motorRight.getCurrentPosition())/2;
            if(gamepad2.dpad_up) {
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
            }
            targetEncoderValue = baseEncoderValue;

            if (previousTargetEncoderValue != targetEncoderValue){
                ArmPosition(targetEncoderValue);
            }

            telemetry.addData("Gamepad 2 Left Stick X", gamepad2.left_stick_y);
            telemetry.addData("Left Motor", motorLeft.getPower());
            telemetry.addData("Right Motor", motorRight.getPower());
            telemetry.addData("Target Position", targetPos);
            telemetry.addData("Current LeftLift Encoder Value", motorLeft.getCurrentPosition());
            telemetry.addData("Current RightLift Encoder Value", motorRight.getCurrentPosition());
            telemetry.addData("Motor Encoder Average", motorEncoderAvg);
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
    public void claw(double posclaw) {
        claw.setPosition(posclaw);
        gamepad1.rumble(500);
        gamepad2.rumble(500);
    }
}

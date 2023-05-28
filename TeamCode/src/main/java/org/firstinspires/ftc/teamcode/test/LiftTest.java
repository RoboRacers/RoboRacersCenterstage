package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
@Disabled
public class LiftTest extends LinearOpMode {

    DcMotorEx motorLeft;
    DcMotorEx motorRight;

    Servo flipbarRight;
    Servo flipbarLeft;
    Servo clawRotator;

    final int extend = 1;
    final int rest = 0;
    final int retract = -1;


    @Override
    public void runOpMode() throws InterruptedException {
        motorLeft = hardwareMap.get(DcMotorEx.class, "LiftLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "LiftRight");

        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flipbarLeft = hardwareMap.get(Servo.class, "fbl");
        flipbarRight = hardwareMap.get(Servo.class, "fbr");
        clawRotator = hardwareMap.get(Servo.class, "crt");

        while (opModeInInit()) {
            flipbarLeft.setPosition(0.5);
            flipbarRight.setPosition(0.5);
        }

        while (!isStopRequested()) {
            if (gamepad2.left_stick_y < -0.5) {
                motorLeft.setPower(-0.1);
                motorRight.setPower(-0.1);
            } else if (gamepad2.left_stick_y > 0.5) {
                motorLeft.setPower(0.1);
                motorRight.setPower(0.1);
            } else if (gamepad2.left_stick_y > 0.5) {
                motorLeft.setPower(0.1);
                motorRight.setPower(0.1);
            } else if(gamepad2.x) {
                clawRotator.setPosition(0);
            } else if(gamepad2.y) {
                clawRotator.setPosition(1);
            } else if(gamepad2.a) {
                flipbarLeft.setPosition(1);
                flipbarRight.setPosition(1);
            } else if(gamepad2.b) {
                flipbarLeft.setPosition(0);
                flipbarRight.setPosition(0);
            }  else if(gamepad2.dpad_up) {

            }

            telemetry.addData("Gamepad 2 Left Stick Y", gamepad2.left_stick_y);
            telemetry.addData("Left Motor", motorLeft.getPower());
            telemetry.addData("Right Motor", motorRight.getPower());
            telemetry.update();
        }
    }

    public void flip(int flipped) {
        // Retract
        if (flipped == -1) {
            flipbarLeft.setPosition(1);
            flipbarRight.setPosition(1);
            clawRotator.setPosition(1);
        }
        // Extend
        else if (flipped == 1) {
            flipbarLeft.setPosition(0);
            flipbarRight.setPosition(0);
            clawRotator.setPosition(0);
        }
        // Rest
        else if (flipped == 0) {
            flipbarLeft.setPosition(0.5);
            flipbarRight.setPosition(0.5);
            clawRotator.setPosition(1);
        }

    }
}

package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;

@TeleOp(name = "Teleop For League Tournament", group = "16481-Power-Play")
public class StateMachines extends LinearOpMode {


    //Turning power play teleOp into state machines
    public enum STATE_CLAW {
        STATE_CLAW_OPEN,
        STATE_CLAW_CLOSE
    }

    public enum STATE_ARM {
        STATE_ARM_LOW,
        STATE_ARM_MED,
        STATE_ARM_HIGH,
        STATE_ARM_MANUAL_UP,
        STATE_ARM_MANUAL_DOWN
    }

    //Setting Current state for each section to desired starting state
    public STATE_CLAW InitCLAW = STATE_CLAW.STATE_CLAW_OPEN;

    public STATE_ARM InitARM = STATE_ARM.STATE_ARM_LOW;

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

    Servo claw;

    @Override
    public void runOpMode(){

        claw = hardwareMap.get(Servo.class, "claw");


        switch (InitCLAW) {
            case STATE_CLAW_CLOSE:
                claw.setPosition(closed);
                gamepad1.rumble(500);
                gamepad2.rumble(500);
                telemetry.addData("Claw Closed", "");
                if (gamepad2.y) {
                    InitCLAW = STATE_CLAW.STATE_CLAW_OPEN;
                    break;
                }

            case STATE_CLAW_OPEN:
                claw.setPosition(open);
                gamepad1.rumble(500);
                gamepad2.rumble(500);
                telemetry.addData("Claw Opened", "");
                if (gamepad2.x) {
                    InitCLAW = STATE_CLAW.STATE_CLAW_CLOSE;
                    break;
                }

            default:
                InitCLAW = STATE_CLAW.STATE_CLAW_OPEN;
        }


        switch (InitARM) {
            case STATE_ARM_LOW:
                if (gamepad1.a) {
                    InitARM = STATE_ARM.STATE_ARM_MED;
                    break;
                }

                else if (gamepad1.b) {
                    InitARM = STATE_ARM.STATE_ARM_HIGH;
                    break;
                }

                else if (0 < gamepad1.right_trigger) {
                    InitARM = STATE_ARM.STATE_ARM_MANUAL_UP;
                    break;
                }

                else if (0 < gamepad1.left_trigger) {
                    InitARM = STATE_ARM.STATE_ARM_MANUAL_DOWN;
                    break;
                }
                // mdkkdk
            case STATE_ARM_MED:
                if (gamepad1.y) {
                    InitARM = STATE_ARM.STATE_ARM_LOW;
                    break;
                }

                else if (gamepad1.b) {
                    InitARM = STATE_ARM.STATE_ARM_HIGH;
                    break;
                }

                else if (0 < gamepad1.right_trigger) {
                    InitARM = STATE_ARM.STATE_ARM_MANUAL_UP;
                    break;
                }

                else if (0 < gamepad1.left_trigger) {
                    InitARM = STATE_ARM.STATE_ARM_MANUAL_DOWN;
                    break;
                }
            case STATE_ARM_HIGH:
                if (gamepad1.y) {
                    InitARM = STATE_ARM.STATE_ARM_LOW;
                    break;
                }
                else if (gamepad1.a) {
                    InitARM = STATE_ARM.STATE_ARM_MED;
                    break;
                }
                else if (0 < gamepad1.right_trigger) {
                    InitARM = STATE_ARM.STATE_ARM_MANUAL_UP;
                    break;
                }
                else if (0 < gamepad1.left_trigger) {
                    InitARM = STATE_ARM.STATE_ARM_MANUAL_DOWN;
                    break;
                }
            case STATE_ARM_MANUAL_UP:
                if (gamepad1.y) {
                    InitARM = STATE_ARM.STATE_ARM_LOW;
                }
                else if (gamepad1.a) {
                    InitARM = STATE_ARM.STATE_ARM_MED;
                    break;
                }
                else if (gamepad1.b) {
                    InitARM = STATE_ARM.STATE_ARM_HIGH;
                    break;
                }
                else if (0 < gamepad1.left_trigger) {
                    InitARM = STATE_ARM.STATE_ARM_MANUAL_DOWN;
                    break;
                }
            case STATE_ARM_MANUAL_DOWN:
                if (gamepad1.y) {
                    InitARM = STATE_ARM.STATE_ARM_LOW;
                }
                else if (gamepad1.a) {
                    InitARM = STATE_ARM.STATE_ARM_MED;
                    break;
                }
                else if (gamepad1.b) {
                    InitARM = STATE_ARM.STATE_ARM_HIGH;
                    break;
                }
                else if (0 < gamepad1.right_trigger) {
                    InitARM = STATE_ARM.STATE_ARM_MANUAL_UP;
                    break;
                }
            default:
                InitARM = STATE_ARM.STATE_ARM_LOW;
        }

        telemetry.update();

    }
}
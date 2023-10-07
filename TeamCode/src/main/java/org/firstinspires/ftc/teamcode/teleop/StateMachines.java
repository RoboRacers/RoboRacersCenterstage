package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;

@TeleOp(name = "Teleop For League Tournament", group = "16481-Power-Play")
public class StateMachines extends LinearOpMode {


    //Turning power play teleOp into state machines
    public enum STATE_INTAKE {
        STATE_INTAKE_ON,
        STATE_INTAKE_OFF
    }

    public enum STATE_ARM {
        STATE_ARM_LOW,
        STATE_ARM_MED,
        STATE_ARM_HIGH,
        STATE_ARM_MANUAL_UP,
        STATE_ARM_MANUAL_DOWN
    }

    //Setting Current state for each section to desired starting state
    public STATE_INTAKE InitINTAKE = STATE_INTAKE.STATE_INTAKE_OFF;

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
    DcMotor intake;

    RobotCore robot;

    @Override
    public void runOpMode(){

        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(DcMotor.class, "intake");

        robot = new RobotCore(hardwareMap, gamepad1, gamepad2);

        while (opModeIsActive()) {
            switch (InitINTAKE) {
                case STATE_INTAKE_OFF:
                    gamepad1.rumble(500);
                    gamepad2.rumble(500);
                    telemetry.addData("Intake is OFF", "");
                    if (gamepad2.y) {
                        InitINTAKE = STATE_INTAKE.STATE_INTAKE_ON;
                    }
                    break;

                case STATE_INTAKE_ON:
                    claw.setPosition(closed);
                    gamepad1.rumble(500);
                    gamepad2.rumble(500);
                    telemetry.addData("Intake is ON", "");
                    if (gamepad2.x) {
                        InitINTAKE = STATE_INTAKE.STATE_INTAKE_OFF;
                        break;
                    }

                default:
                    InitINTAKE = STATE_INTAKE.STATE_INTAKE_OFF;
            }

        telemetry.update();

        }
    }
}
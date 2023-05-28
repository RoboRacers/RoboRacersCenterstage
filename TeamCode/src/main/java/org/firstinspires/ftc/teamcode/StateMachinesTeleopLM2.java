package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
/*
@TeleOp(name = "Teleop For League Tournament", group = "16481-Power-Play")
public class StateMachinesTeleopLM2 extends LinearOpMode {
    /*
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

    public enum STATE_ARM {
        STATE_ARM_LOW,
        STATE_ARM_MED,
        STATE_ARM_HIGH,
        STATE_ARM_MANUAL_UP,
        STATE_ARM_MANUAL_DOWN
    }

    //Setting Current state for each section to desired starting state
    public AutoopStateMachines.STATE_CLAW InitCLAW = AutoopStateMachines.STATE_CLAW.STATE_CLAW_OPEN;

     while runOpMode{
        switch (InitCLAW) {
            case (if gamepad2.x):
                InitCLAW = AutoopStateMachines.STATE_CLAW.STATE_CLAW_CLOSE;
                break;
        }
            switch (STATE_CLAW.STATE_CLAW_OPEN) {
                case (if gamepad2.x):
                    InitCLAW = AutoopStateMachines.STATE_CLAW.STATE_CLAW_CLOSE;
                    break;
            }
        switch (STATE_CLAW.STATE_CLAW_CLOSE) {
            case (if gamepad2.y):
                InitCLAW = AutoopStateMachines.STATE_CLAW.STATE_CLAW_OPEN;
                break;
            }
        }
     */


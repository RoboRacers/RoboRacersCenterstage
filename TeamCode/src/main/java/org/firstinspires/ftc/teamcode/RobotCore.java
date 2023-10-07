package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;

/**
 * This class contains the declaration and setup for all motors and cameras on the robot.
 * To use this class in an opmode, declare:
 * RobotCore robot = new RobotCore(hardwareMap);
 * You can call motors and such through:
 * robot.componentName.command();
 */

public class RobotCore {

    // Declare the motors and servos
    public SampleMecanumDrive drive;
    public Deposit deposit;

    public Gamepad gamepad1;
    public Gamepad gamepad2;

    public HardwareMap hardwareMap;

    // Constructor, run during initialization
    public RobotCore(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2){


        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        deposit = new Deposit(hardwareMap);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        this.hardwareMap = hardwareMap;
    }


}

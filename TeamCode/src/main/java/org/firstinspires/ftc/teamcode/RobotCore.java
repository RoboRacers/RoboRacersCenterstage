package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;

/**
 * This class contains the declaration and setup for all motors and cameras on the robot.
 * To use this class in an opmode, declare:
 * RobotCore robot = new RobotCore(hardwareMap);
 * You can call motors and such through:
 * robot.componentName.command();
 */

public class RobotCore {


    // Declare the motors and servos
    public DcMotorEx motorLeft;
    public DcMotorEx motorRight;
    public Servo claw;
    public SampleMecanumDrive drive;
    public OpenCvCamera camera;

    // Initialization
    public RobotCore(HardwareMap hardwareMap){

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


}

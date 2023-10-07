package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotCore;

@TeleOp(name = "Distance_Slow", group = "16481-Centerstage")
public class Distance_Slow extends LinearOpMode {

    RobotCore robot;
    double driveSensitivity;
    double turnSensitivity;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotCore(hardwareMap, gamepad1, gamepad2);
        driveSensitivity = 0.5;
        turnSensitivity = 0.5;
        //runs once after init

        while (opModeInInit()) { //runs continusly after the once part
            //stuff goes here hehehehehehe

        }
        //runs when start is pressed
        //runs once between loops

        while (!isStopRequested()) { //runs contiuosly after start is pressed until stop
        //also stuff goes here hehehehehehe

            robot.drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y*driveSensitivity, -gamepad1.left_stick_x*driveSensitivity, -gamepad1.right_stick_x*turnSensitivity));
             



        }
    }
}

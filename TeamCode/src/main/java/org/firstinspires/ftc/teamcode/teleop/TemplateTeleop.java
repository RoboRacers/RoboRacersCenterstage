package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotCore;

@TeleOp(name = "Templete Teleop", group = "16481-Power-Play")
public class TemplateTeleop extends LinearOpMode {

    RobotCore robot = new RobotCore(hardwareMap);

    int baseEncoderValue;
    final int liftLow = 0;
    final int liftHigherThanLow = -750;
    final int liftMid = -1075;
    final int liftHigh = -1350;

    double driveSensitivity = .5;
    double turnSensitivity = .75;
    double liftSpeed = .5;


    final double closed = 0.25;
    final double open = 0;
    @Override
    public void runOpMode() throws InterruptedException {



        while (opModeInInit()) {
            robot.setClawPos(open);
        }

        while (!isStopRequested()) {

            robot.drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y*driveSensitivity, -gamepad1.left_stick_x*driveSensitivity, -gamepad1.right_stick_x*turnSensitivity));
            robot.drive.update();

            if(gamepad2.right_bumper) {
                robot.claw.setPosition(closed);
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            } else if(gamepad2.left_bumper){
                robot.claw.setPosition(open);
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            } else if(gamepad2.dpad_up) {
                // Set arm Position to High
                baseEncoderValue = liftHigh;
            } else if(gamepad2.dpad_down) {
                // Set arm Position to Low
                robot.setArmPos(liftLow, liftSpeed);
            }else if(gamepad2.dpad_left) {
                // Set arm Position to Medium
                baseEncoderValue = liftMid;
            }else if(gamepad2.dpad_right) {
                // Set arm Position to a bit lower than High
                robot.setArmPos(liftHigherThanLow, liftSpeed);
            }



            // Telemetry
            telemetry.addData("Roboracers Teleop for League Tournament", "");
            telemetry.addData("Gamepad 2 Left Stick Y", gamepad2.left_stick_y);
            telemetry.addData("Left Motor Power", robot.motorLeft.getPower());
            telemetry.addData("Right Motor Power", robot.motorRight.getPower());
            telemetry.addData("Left Motor Encoder Value", robot.motorLeft.getCurrentPosition());
            telemetry.addData("Right Motor Encoder Value", robot.motorRight.getCurrentPosition());
            telemetry.update();

        }
    }
}

package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotCore;

@Config
@TeleOp(name = "Teleop for LM3", group = "16481-Centerstage")
public class TeleopLM3 extends LinearOpMode {

    RobotCore robot;

    public static double speedMultiplier = .7;
    public static double strafeMultiplier = .8;
    public static double retractionSpeed = 0.5;
    public static double extensionSpeed = 1;
    public static double feedforward = 0.0;
    public static boolean liftRetractionOverride = false;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotCore(hardwareMap);

        Gamepad previousGamepad1 = gamepad1;
        Gamepad previousGamepad2 = gamepad2;

        while (opModeInInit()) {
        }

        while (!isStopRequested()) {

            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y * speedMultiplier,
                            -gamepad1.left_stick_x * speedMultiplier, //imperfect strafing fix, must be tuned for new drivetrain
                            gamepad1.right_stick_x * strafeMultiplier
                    )
            );

            // Slides control
            if (gamepad2.right_stick_y > 0.1 && gamepad2.left_bumper) {
                robot.slides.setPower(-gamepad2.right_stick_y);
            } else if (gamepad2.right_stick_y < -0.1) {
                robot.slides.setPower(-gamepad2.right_stick_y);
            } else if (gamepad2.right_stick_y > 0.1) {
                robot.slides.setPower(-gamepad2.right_stick_y*.4);
            } else {
                robot.slides.setPower(0);
            }

            // Intake control
            if (gamepad1.right_trigger > 0.1) {
                robot.intake.setIntakePower(1);
            } else if (gamepad1.left_trigger > 0.1) {
                robot.intake.setIntakePower(-1);
            } else {
                robot.intake.setIntakePower(0);
            }

            // Lock deposit
            if (gamepad2.right_bumper) {
                robot.intake.engageLock(true, true);
            }

            // Release deposit
            if (gamepad2.right_trigger > 0.7) {
                robot.intake.clearLowerLock();
            }
            if (gamepad2.left_trigger > 0.7) {
                robot.intake.clearHigherLock();
            }

            if (gamepad2.dpad_up) {
                robot.intake.flipDeposit();
            } else if (gamepad2.dpad_down) {
                robot.intake.flipIntake();
            }


            robot.update();

            previousGamepad1 = gamepad1;
            previousGamepad2 = gamepad2;

            telemetry.addLine("\uD83C\uDFCE RoboRacers Teleop for League Meet 3");
            telemetry.update();
        }
    }
}

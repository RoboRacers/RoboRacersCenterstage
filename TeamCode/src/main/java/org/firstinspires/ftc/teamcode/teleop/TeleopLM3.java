package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.statemachines.SlidesSM;

@Config
@TeleOp(name = "Teleop for LM3", group = "16481-Centerstage")
public class TeleopLM3 extends LinearOpMode {

    RobotCore robot;

    public static double speedMultiplier = .7;
    public static double strafeMultiplier = .6;
    public static double turnMultiplier = .8;
    public static double retractionSpeed = .5;
    public static double extensionSpeed = 1;
    public static double rtpLockPower = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotCore(hardwareMap);

        Gamepad previousGamepad1 = gamepad1;
        Gamepad previousGamepad2 = gamepad2;

        boolean test = false;

        robot.slides.statemachine.transition(
                SlidesSM.EVENT.ENABLE_MANUAL
        );

        while (opModeInInit()) {
        }

        while (!isStopRequested()) {

            // Drive control
            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y * speedMultiplier,
                            gamepad1.left_stick_x * strafeMultiplier,
                            -gamepad1.right_stick_x * turnMultiplier
                    )
            );

            // Slides control
            if (gamepad2.right_stick_y > 0.1 && gamepad2.left_bumper) {
                robot.slides.setManualPower(-gamepad2.right_stick_y);
            } else if (gamepad2.right_stick_y > 0.1) {
                robot.slides.setManualPower(-gamepad2.right_stick_y*extensionSpeed);
            } else if (gamepad2.right_stick_y < -0.1) {
                robot.slides.setManualPower(-gamepad2.right_stick_y*retractionSpeed);
            } else {
                robot.slides.setTargetPosition(
                        robot.slides.getCurrentPosition()
                );
                robot.slides.statemachine.transition(
                        SlidesSM.EVENT.ENABLE_RTP
                );
                robot.slides.setPower(rtpLockPower);
            }

            // Intake control
            if (gamepad1.right_trigger > 0.1) {
                robot.intake.setIntakePower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                robot.intake.setIntakePower(-gamepad1.left_trigger);
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

            // Flip deposit
            if (gamepad2.dpad_up) {
                robot.intake.flipDeposit();
            } else if (gamepad2.dpad_down) {
                robot.intake.flipIntake();
            } else if (!gamepad2.dpad_left && previousGamepad2.dpad_left) {
                robot.intake.incrementFlip(-0.1);
                telemetry.addLine("Increment Down");
                test = true;
            } else if (!gamepad2.dpad_right && previousGamepad2.dpad_right) {
                robot.intake.incrementFlip(0.1);
                telemetry.addLine("Increment Down");
                test = true;
            }

            // Update all state machines
            robot.update();

            // Previous gamepad for edge detection
            previousGamepad1 = gamepad1;
            previousGamepad2 = gamepad2;

            // Telemetry
            telemetry.addLine("\uD83C\uDFCE RoboRacers Teleop for League Meet 3");
            telemetry.addData("Slides RunMode", robot.slides.statemachine.getState());
            telemetry.addData("Slides Power", robot.slides.leftmotor.getPower());
            telemetry.addData("Slides Target Position", robot.slides.getTargetPosition());
            telemetry.addData("Intake Power", robot.intake.intakeMotor.getPower());
            telemetry.addLine(String.valueOf(test));
            telemetry.update();
        }
    }
}

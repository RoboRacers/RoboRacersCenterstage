package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.statemachines.IntakeSM;

@Config
@TeleOp(name = "Teleop for LM2", group = "16481-Centerstage")
public class TeleopLM2 extends LinearOpMode {

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
                            -gamepad1.left_stick_y * speedMultiplier,
                            -gamepad1.left_stick_x * speedMultiplier, //imperfect strafing fix, must be tuned for new drivetrain
                            -gamepad1.right_stick_x * strafeMultiplier
                    )
            );

            // Claw control
            if (gamepad2.dpad_down) {
                robot.intake.statemachine.transition(
                        IntakeSM.EVENT.CLOSED_WITH_PIXEL
                );
            } else if (gamepad2.dpad_left) {
                robot.intake.statemachine.transition(
                        IntakeSM.EVENT.CLOSING_FOR_PIXEL
                );
            } else if (gamepad2.dpad_right) {
                robot.intake.statemachine.transition(
                        IntakeSM.EVENT.OPEN_FOR_PIXEL
                );
            } else if (gamepad2.dpad_up) {
                robot.intake.statemachine.transition(
                        IntakeSM.EVENT.EXTEND_WITH_PIXEL
                );
            }

            if (gamepad2.right_bumper) {
                robot.intake.statemachine.transition(
                        IntakeSM.EVENT.OPEN_CLAW
                );
            } else if (gamepad2.left_bumper) {
                robot.intake.statemachine.transition(
                        IntakeSM.EVENT.CLOSE_CLAW
                );
            }

            if (gamepad2.cross){
                robot.drone.fireDrone(true);
            }

            if (gamepad2.a && !previousGamepad2.a) {
                if (liftRetractionOverride) {
                    liftRetractionOverride = false;
                } else {
                    liftRetractionOverride = true;
                }
            }

            /* Slides control

            if (gamepad2.right_stick_y > 0.1 && !liftRetractionOverride) {
                robot.slides.setManualPower(-gamepad2.right_stick_y*retractionSpeed);
            } else if (gamepad2.right_stick_y > 0.1 && liftRetractionOverride) {
                robot.slides.setManualPower(-gamepad2.right_stick_y);
            } else if (gamepad2.right_stick_y < -0.1) {
                robot.slides.setManualPower(-gamepad2.right_stick_y*extensionSpeed);
            } else {
                robot.slides.setManualPower(0+feedforward);
            }
            */

            if (gamepad2.right_stick_y > 0.2 && gamepad2.right_trigger < 0.1) {
                robot.slides.setManualPower(-gamepad2.right_stick_y*retractionSpeed);
            } else if (gamepad2.right_stick_y > 0.2 && gamepad2.right_trigger >= 0.1) {
                robot.slides.setManualPower(-gamepad2.right_stick_y);
                telemetry.addLine("Lift TURBO Mode");
            } else if (gamepad2.right_stick_y < -0.2) {
                robot.slides.setManualPower(-gamepad2.right_stick_y*extensionSpeed);
            } else {
                robot.slides.setManualPower(0);
            }

            robot.update();

            previousGamepad1 = gamepad1;
            previousGamepad2 = gamepad2;

            telemetry.addLine("\uD83C\uDFCE RoboRacers Teleop for League Meet 2");

            telemetry.update();
        }
    }
}

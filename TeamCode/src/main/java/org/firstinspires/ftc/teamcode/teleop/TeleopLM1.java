package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.statemachines.IntakeSM;

@TeleOp(name = "Teleop for LM1", group = "16481-Centerstage")
public class TeleopLM1 extends LinearOpMode {

    RobotCore robot;

    double speedMultiplier = .7;
    double strafeMultiplier = .8;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotCore(hardwareMap);

        while (opModeInInit()) {
        }

        while (!isStopRequested()) {

            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * speedMultiplier,
                            gamepad1.left_stick_x * speedMultiplier, //imperfect strafing fix, must be tuned for new drivetrain
                            gamepad1.right_stick_x * strafeMultiplier
                    )
            );

            if (gamepad2.dpad_down) {
                robot.intake.setIntake(.1, .475, .8);
            } else if (gamepad2.dpad_left) {
                robot.intake.setIntake(.27,0.55,.8);
            } else if (gamepad2.dpad_right) {
                robot.intake.setIntake(.1,.475,.8);
            } else if (gamepad2.dpad_up) {
                robot.intake.setIntake(.1,0.05, .0);
            }

            if (gamepad2.right_bumper) {
                robot.intake.claw.setPosition(0.3);
            }

            if (gamepad2.right_stick_y > 0.1) {
                robot.slides.setManualPower(-gamepad2.right_stick_y*1);
            } else if (gamepad2.right_stick_y < -0.1) {
                robot.slides.setManualPower(-gamepad2.right_stick_y*.7);
            } else {
                robot.slides.setManualPower(0);
            }

            robot.update();
        }
    }
}

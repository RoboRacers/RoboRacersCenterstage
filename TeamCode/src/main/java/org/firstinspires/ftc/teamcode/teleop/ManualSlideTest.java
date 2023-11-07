package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotCore;

@TeleOp(name = "Manual Slides Test", group = "16481-Centerstage")
public class ManualSlideTest extends LinearOpMode {

    DcMotor slideMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");

        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (opModeInInit()) {

        }


        while (!isStopRequested()) {

            if (gamepad1.left_stick_y > 0.1) {
                slideMotor.setPower(0.5);
            } else if (gamepad1.left_stick_y < -0.1) {
                slideMotor.setPower(-0.5);
            } else  {
                slideMotor.setPower(0);
            }

        }
    }
}

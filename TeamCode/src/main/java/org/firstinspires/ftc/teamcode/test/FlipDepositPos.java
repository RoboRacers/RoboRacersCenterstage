package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.RobotCore;


@TeleOp(name = "Flip Deposit Pos", group = "16481-Template")
public class FlipDepositPos extends LinearOpMode {
    RobotCore robot;

    ServoImplEx flipLeft;
    ServoImplEx flipRight;

    @Override
    public void runOpMode() throws InterruptedException {

        flipLeft = hardwareMap.get(ServoImplEx.class, "flipLeft");
        flipRight = hardwareMap.get(ServoImplEx.class, "flipRight");
        flipLeft.setDirection(Servo.Direction.REVERSE);

        while (opModeInInit()) {

        }

        while (opModeIsActive()) {
            flipLeft.setPosition(1);
            flipRight.setPosition(0.92);

            telemetry.addData("Left servo pos", flipLeft.getPosition());
            telemetry.addData("Right servo pos", flipRight.getPosition());
            telemetry.update();
        }
    }
}

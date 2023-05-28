package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "Claw Test")
public class ClawTest extends LinearOpMode {
    Servo claw;

    double open = 0.0;
    double close = 0.4;


    @Override
    public void runOpMode() {
        claw = hardwareMap.get(Servo.class, "claw");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.dpad_up) {
                claw.setPosition(open);
            } else if(gamepad1.dpad_down) {
                claw.setPosition(close);
            }
        }
    }
}
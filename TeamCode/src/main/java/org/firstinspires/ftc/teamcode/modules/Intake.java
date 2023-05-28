package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "IntakeStates")
public class Intake extends LinearOpMode {
    public enum IStates{
        open,
        close,
        idle
    }
    public double servoOpen = .5;
    public double servoClose = 0;
    public IStates IntakeStates = IStates.idle;

    Servo claw;

    @Override
    public void runOpMode() {

        claw = hardwareMap.get(Servo.class, "claw");

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()){
            switch(IntakeStates) {
                case open:

                    setIntakePosition(claw, servoOpen);

                    break;
                case close:

                    setIntakePosition(claw, servoClose);

                    break;

                default:
                    break;
            }
        }
    }

    public void setIntakePosition(Servo claw, double x) {
        claw.setPosition(x);
    }
}

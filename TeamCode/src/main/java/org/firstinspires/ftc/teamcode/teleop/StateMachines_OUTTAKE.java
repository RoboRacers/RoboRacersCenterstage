package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotCore;

@TeleOp(name = "Teleop For League Tournament output", group = "16481-Power-Play")
public class StateMachines_OUTTAKE extends LinearOpMode {


    //Turning power play teleOp into state machines

    public enum STATE_OUTTAKE {
        STATE_OUTTAKE_EXTENDED,
        STATE_OUTTAKE_CONDENSED,
        STATE_OUTTAKE_DEPOSIT //There is no close state for the enum because it automatically happens in the condensed state and after the open state is done
    }

    STATE_OUTTAKE currentState;

    public STATE_OUTTAKE getState() {
        return currentState;
    }

    //Setting Current state for each section to desired starting state
    public STATE_OUTTAKE InitOUTTAKE = STATE_OUTTAKE.STATE_OUTTAKE_CONDENSED;

    Servo claw;
    DcMotor intake;

    RobotCore robot;

    @Override
    public void runOpMode(){

        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(DcMotor.class, "intake");

        robot = new RobotCore(hardwareMap, gamepad1, gamepad2);

        while (opModeIsActive()) {
            switch (InitOUTTAKE) {
                case STATE_OUTTAKE_CONDENSED:
                    telemetry.addData("Outtake is NOT Extended", "");
                    if (gamepad2.a) {
                        InitOUTTAKE = STATE_OUTTAKE.STATE_OUTTAKE_EXTENDED;
                    }
                    else if (0 < gamepad2.right_trigger) {
                        InitOUTTAKE = STATE_OUTTAKE.STATE_OUTTAKE_DEPOSIT;
                    }
                    break;

                case STATE_OUTTAKE_EXTENDED:
                    telemetry.addData("Outtake is EXTENDED", "");
                    if (gamepad2.b) {
                        InitOUTTAKE = STATE_OUTTAKE.STATE_OUTTAKE_CONDENSED;
                    }
                    if (0 < gamepad2.right_trigger) {
                        InitOUTTAKE = STATE_OUTTAKE.STATE_OUTTAKE_DEPOSIT;
                    }
                    break;

                case STATE_OUTTAKE_DEPOSIT:
                    telemetry.addData("Outtake is DEPOSITED", "");
                    if (gamepad2.b) {
                        InitOUTTAKE = STATE_OUTTAKE.STATE_OUTTAKE_CONDENSED;
                    }
                    else if (gamepad2.a) {
                        InitOUTTAKE = STATE_OUTTAKE.STATE_OUTTAKE_EXTENDED;
                    }
                    break;

                default:
                    InitOUTTAKE = STATE_OUTTAKE.STATE_OUTTAKE_CONDENSED;
            }

        telemetry.update();

        }
    }
}
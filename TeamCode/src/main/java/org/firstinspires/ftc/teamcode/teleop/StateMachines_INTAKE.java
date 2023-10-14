package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotCore;

@TeleOp(name = "Teleop For League Tournament input", group = "16481-Power-Play")
public class StateMachines_INTAKE extends LinearOpMode {


    //Turning power play teleOp into state machines
    public enum STATE_INTAKE {
        STATE_INTAKE_ON,
        STATE_INTAKE_OFF
    }

    STATE_INTAKE currentState;

    public STATE_INTAKE getState() {
        return currentState;
    }


    //Setting Current state for each section to desired starting state
    public STATE_INTAKE InitINTAKE = STATE_INTAKE.STATE_INTAKE_OFF;

    Servo claw;
    DcMotor intake;

    RobotCore robot;

    @Override
    public void runOpMode(){

        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(DcMotor.class, "intake");

        robot = new RobotCore(hardwareMap, gamepad1, gamepad2);

        while (opModeIsActive()) {
            switch (InitINTAKE) {
                case STATE_INTAKE_OFF:
                    telemetry.addData("Intake is OFF", "");
                    if (gamepad2.y) {
                        InitINTAKE = STATE_INTAKE.STATE_INTAKE_ON;
                    }
                    break;

                case STATE_INTAKE_ON:
                    telemetry.addData("Intake is ON", "");
                    if (gamepad2.x) {
                        InitINTAKE = STATE_INTAKE.STATE_INTAKE_OFF;
                    }
                    break;

                default:
                    InitINTAKE = STATE_INTAKE.STATE_INTAKE_OFF;
            }

        telemetry.update();

        }

        if (gamepad2.x) {
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }

        else if (gamepad2.y) {
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }

    }
}
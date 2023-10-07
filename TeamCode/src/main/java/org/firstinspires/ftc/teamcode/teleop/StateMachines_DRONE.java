package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotCore;

@TeleOp(name = "Teleop For League Tournament", group = "16481-Power-Play")
public class StateMachines_DRONE extends LinearOpMode {


    //Turning power play teleOp into state machines
    public enum STATE_INTAKE {
        STATE_INTAKE_ON,
        STATE_INTAKE_OFF
    }

    public enum STATE_OUTTAKE {
        STATE_OUTTAKE_EXTENDED,
        STATE_OUTTAKE_CONDENSED,
        STATE_OUTTAKE_DEPOSIT //There is no close state for the enum because it automatically happens in the condensed state and after the open state is done
    }

    public enum STATE_DRONE {
        STATE_DRONE_LOADED,
        STATE_DRONE_LAUNCHED
    }

    //Setting Current state for each section to desired starting state
    public STATE_INTAKE InitINTAKE = STATE_INTAKE.STATE_INTAKE_OFF;
    public STATE_OUTTAKE InitOUTTAKE = STATE_OUTTAKE.STATE_OUTTAKE_CONDENSED;
    public STATE_DRONE InitDRONE = STATE_DRONE.STATE_DRONE_LOADED;


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
            }
            switch (InitDRONE) {
                case STATE_DRONE_LOADED:
                    telemetry.addData("Drone is LOADED", "");
                    if (gamepad2.left_bumper) {
                        InitDRONE = STATE_DRONE.STATE_DRONE_LAUNCHED;
                    }
                    break;

                case STATE_DRONE_LAUNCHED:
                    telemetry.addData("Drone is LAUNCHED", "");
                    break;
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
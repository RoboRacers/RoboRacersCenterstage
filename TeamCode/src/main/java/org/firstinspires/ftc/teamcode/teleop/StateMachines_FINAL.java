package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotCore;

@TeleOp(name = "Teleop For League Tournament SM", group = "16481-Power-Play")
public class StateMachines_FINAL extends LinearOpMode {


    //Turning power play teleOp into state machines
    public enum STATE_INTAKE {
        INTAKE_RUNNING,
        INTAKE_NOT_RUNNING
    }

    public enum STATE_DEPOSIT {
        DEPOSIT_EXTENDED,
        DEPOSIT_RETRACTED,
        DEPOSIT_DEPOSITED //There is no close state for the enum because it automatically happens in the condensed state and after the open state is done
    }

    public enum STATE_DRONE {
        DRONE_LOADED,
        DRONE_LAUNCHED
    }

    public enum STATE_EVENT {
        GAME_START,
        EXTENDING,
        RETRACTING,
        DEPOSITING,
        ON_INTAKE,
        OFF_INTAKE,
        DRONE_LOADING,
        DRONE_LAUNCHING
    }

    //Setting Current state for each section to desired starting state
    public STATE_INTAKE InitINTAKE = STATE_INTAKE.INTAKE_NOT_RUNNING;
    public STATE_DEPOSIT InitDEPOSIT = STATE_DEPOSIT.DEPOSIT_RETRACTED;
    public STATE_DRONE InitDRONE = STATE_DRONE.DRONE_LOADED;
    public STATE_EVENT InitEVENT = STATE_EVENT.GAME_START;

    STATE_EVENT currentState;
    public StateMachines_FINAL.STATE_EVENT getState(){
    return currentState;
    }

    int previousTargetEncoderValue;
    int targetEncoderValue;
    int previousCommonModifier;
    int commonModifier = 0;
    int baseEncoderValue;
    final int liftLow = 0;
    final int liftHigherThanLow = -750;
    final int liftMid = -1075;
    final int liftHigh = -1350;

    double driveSensitivity = .5;
    double turnSensitivity = .75;
    double liftSpeed = .5;

    int motorEncoderAvg;

    final double closed = 0.25;
    final double open = 0;

    Servo claw;
    DcMotor intake;

    RobotCore robot;

    public void transition(StateMachines_FINAL.STATE_EVENT event){
        swtich (event) {
            case GAME_START:
                currentState = StateMachines_FINAL.
        }
    }

    @Override
    public void runOpMode(){

        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(DcMotor.class, "intake");

        robot = new RobotCore(hardwareMap, gamepad1, gamepad2);

        while (opModeIsActive()) {
            if (gamepad1.a) {

            }

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
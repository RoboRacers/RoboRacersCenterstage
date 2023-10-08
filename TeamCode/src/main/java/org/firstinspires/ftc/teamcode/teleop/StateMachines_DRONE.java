package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotCore;

@TeleOp(name = "Teleop For League Tournament", group = "16481-Power-Play")
public class StateMachines_DRONE extends LinearOpMode {


    //Turning power play teleOp into state machines

    public enum STATE_DRONE {
        STATE_DRONE_LOADED,
        STATE_DRONE_LAUNCHED
    }

    STATE_DRONE currentState;

    public STATE_DRONE getState() {
        return currentState;
    }

    //Setting Current state for each section to desired starting state

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

                default:
                    InitDRONE = STATE_DRONE.STATE_DRONE_LOADED;
            }

        telemetry.update();

        }
    }
}
package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.statemachines.IntakeSM;
import org.firstinspires.ftc.teamcode.modules.subsystems.Intake;

@Config
@TeleOp(name = "Intake Subsystem Test", group = "Test")
public class IntakeSubsystemTest extends LinearOpMode {

    Intake intake;

    @Override
    public void runOpMode(){

        intake = new Intake(hardwareMap);

        while (opModeInInit()){

        }

        Gamepad prevGamepad1= gamepad1;

        while (opModeIsActive()){



            if (gamepad1.a) {
                intake.statemachine.transition(
                        IntakeSM.EVENT.START_INTAKE
                );
                telemetry.addLine("Action run!");
            }

            if (gamepad1.left_bumper) {
                intake.intakeMotor.setPower(.6);
            } else if (gamepad1.right_bumper) {
                intake.intakeMotor.setPower(-.6);
            } else intake.intakeMotor.setPower(0);

            telemetry.addData("Current Intake State", intake.statemachine.getState());
            telemetry.update();
            intake.update();
        }
    }

}

package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.subsystems.Intake;

@Config
@TeleOp(name = "Intake Subsystem Test", group = "Test")
public class IntakeSubsystemTest extends LinearOpMode {

    Intake intake;

    public static double stage1pos = 0.5;

    public static double stage2pos = 0.5;

    public static double clawPos = 0.0;

    @Override
    public void runOpMode(){

        intake = new Intake(hardwareMap);

        while (opModeInInit()){

        }

        while (opModeIsActive()){

            intake.setIntake(clawPos, stage1pos, stage2pos);
            telemetry.update();

        }
    }

}

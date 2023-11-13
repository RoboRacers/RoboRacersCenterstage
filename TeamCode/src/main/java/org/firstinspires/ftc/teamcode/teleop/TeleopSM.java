package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.statemachines.IntakeSM;
import org.firstinspires.ftc.teamcode.modules.statemachines.LauncherSM;
import org.firstinspires.ftc.teamcode.modules.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.modules.subsystems.Intake;
import org.firstinspires.ftc.teamcode.modules.subsystems.SlidesSubsystem;

@TeleOp(name = "Teleop For State Machines", group = "16481-Power-Play")
public class TeleopSM extends LinearOpMode {

    RobotCore robot;
Launcher launcher;
Intake intake;




    @Override
    public void runOpMode(){

        robot = new RobotCore(hardwareMap);

        while (opModeInInit()){
            launcher.statemachine.transition(LauncherSM.EVENT.GAME_START);
            intake.statemachine.transition(IntakeSM.EVENT.GAME_START);
        }
        while (opModeIsActive()){
            if (gamepad1.circle){
                robot.launcher.statemachine.transition(LauncherSM.EVENT.DRONE_LAUNCH_BUTTON_PRESSED);
                telemetry.addData("Drone is being launched", "");
            }
            else if(gamepad1.cross){
                robot.launcher.statemachine.transition(LauncherSM.EVENT.DRONE_RETRACT_BUTTON_PRESSED);
                telemetry.addData("Drone launcher retracted", "");
            }
            //below isn't finished and needs to be finished at sometime(soon maybe)
            else if (gamepad1.dpad_up){
                //robot.
                telemetry.addData("Intake Extended", "");
            }
            else if (gamepad2.b){
                robot.intake.statemachine.transition(IntakeSM.EVENT.RETRACT_WITH_PIXEL);
                telemetry.addData("Intake Retracted", "");
            }
            else if (gamepad2.right_bumper){
                robot.intake.statemachine.transition(IntakeSM.EVENT.GRABBING_PIXEL);
                telemetry.addData("Claw Closed", "");
            }
            else if (gamepad2.left_bumper){
                robot.intake.statemachine.transition(IntakeSM.EVENT.OPEN_CLAW);
                telemetry.addData("Claw Opened", "");
            }
        }
telemetry.update();
    }
}
            // Telemetry
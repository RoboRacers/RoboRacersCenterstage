package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.statemachines.IntakeSM;
import org.firstinspires.ftc.teamcode.modules.statemachines.LauncherSM;
import org.firstinspires.ftc.teamcode.modules.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.modules.subsystems.Intake;

@TeleOp(name = "Teleop For League Tournament", group = "16481-Power-Play")
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
            if (gamepad2.dpad_up){
                robot.launcher.statemachine.transition(LauncherSM.EVENT.DRONE_LAUNCH_BUTTON_PRESSED);
                telemetry.addData("Drone is being launched", "");
            }

            else if (gamepad2.a){
                robot.intake.statemachine.transition(IntakeSM.EVENT.EXTEND_TO_PIXEL);
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
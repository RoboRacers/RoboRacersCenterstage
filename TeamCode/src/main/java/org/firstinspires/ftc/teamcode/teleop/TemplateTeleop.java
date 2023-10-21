package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotCore;

@TeleOp(name = "Template Teleop", group = "16481-Centerstage")
public class TemplateTeleop extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {

        RobotCore robot = new RobotCore(hardwareMap);

        while (opModeInInit()) {
        }

        while (!isStopRequested()) {


        }
    }
}

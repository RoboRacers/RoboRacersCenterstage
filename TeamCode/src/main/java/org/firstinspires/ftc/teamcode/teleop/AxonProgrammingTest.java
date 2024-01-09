package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.RobotCore;


@TeleOp(name = "Axon Programming Test", group = "16481-Template")
public class AxonProgrammingTest extends LinearOpMode {
    RobotCore robot;
    Servo axon1;

    double axonValue = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotCore(hardwareMap);
        axon1 = hardwareMap.get(Servo.class, "axon1");

        while (opModeInInit()) {
            axon1.setPosition(0);
        }

        while (!isStopRequested()) {
            axon1.setPosition(axonValue);
            telemetry.addData("Axon Value", axon1.getPosition());
            telemetry.update();
            robot.update();
        }
    }
}

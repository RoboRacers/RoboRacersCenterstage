package org.firstinspires.ftc.teamcode.test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.modules.drive.MecanumDrive;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Ultrasonic Proximity Test", group = "Test")
public class ProximtyTest extends LinearOpMode {

    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        boolean robotInRange = false;

        MecanumDrive.Side side = MecanumDrive.Side.RIGHT;

        drive = new MecanumDrive(hardwareMap);

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        timer.reset();

        while (!isStopRequested()) {

            if (gamepad1.a) {
                side = MecanumDrive.Side.RIGHT;
            } else if (gamepad1.b) {
                side = MecanumDrive.Side.LEFT;
            } else if (gamepad1.x) {
                timer.reset();
            }


            switch (side) {
                case RIGHT:
                    double rightDistance = drive.rightUltrasonic.getDistance(DistanceUnit.INCH);
                    if (rightDistance < 30) {
                        robotInRange = true;
                        break;
                    } else {
                        robotInRange = false;
                    }
                    if (timer.time(TimeUnit.MILLISECONDS) > 7000) {
                        telemetry.addLine("Timed out!");
                    }
                    break;
                case LEFT:
                    double leftDistance = drive.leftUltrasonic.getDistance(DistanceUnit.INCH);
                    if (leftDistance < 30) {
                        robotInRange = true;
                        break;
                    } else {
                        robotInRange = false;
                    }
                    if (timer.time(TimeUnit.MILLISECONDS) > 7000) {
                        telemetry.addLine("Timed out!");
                    }
                    break;
            }

            telemetry.addData("Robot in Range", robotInRange);
            telemetry.update();

        }
    }
}



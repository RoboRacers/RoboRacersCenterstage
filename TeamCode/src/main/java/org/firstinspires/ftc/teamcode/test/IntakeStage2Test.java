package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.List;

@TeleOp(name = "Intake Stage 2 Test", group = "Test")
public class IntakeStage2Test extends LinearOpMode {

    Servo leftStage2;
    Servo rightStage2;

    double Increment = 0.001;
    double MINPOS = 0.0;
    double MAXPOS = 1.0;


    double position3 = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        rightStage2 = hardwareMap.get(Servo.class, "stage2right");
        leftStage2 = hardwareMap.get(Servo.class, "stage2left");

        leftStage2.setDirection(Servo.Direction.REVERSE);

        while (opModeInInit()) {
        }


        //leftStage1.setPosition(0.5);

        while (!isStopRequested()) {

            if(gamepad1.dpad_up){
                position3 += Increment;
                if (position3 >= MAXPOS){
                    position3 = MAXPOS;
                    position3 = MAXPOS - 0.1;
                }
            }
            else if(gamepad1.dpad_down){
                position3 -= Increment;
                if (position3 <= MINPOS){
                    position3 = MINPOS;
                    position3 = MINPOS + 0.1;
                }
            }

            rightStage2.setPosition(position3);
            leftStage2.setPosition(position3);

            // Telemetry
            telemetry.addData("Right Servo Position", rightStage2.getPosition());
            telemetry.addData("Left Servo Position", leftStage2.getPosition());
            telemetry.update();

        }
    }
}

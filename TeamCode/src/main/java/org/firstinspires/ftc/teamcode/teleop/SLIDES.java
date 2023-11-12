package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Slides_TEST_ANISH", group = "16481-Power-Play")
public class SLIDES extends LinearOpMode {

//Servo drone;
DcMotorEx slides1;
DcMotorEx slides2;

ElapsedTime timer = new ElapsedTime();

double slidesPower1 = 0.0;
double slidesPower2 = 0.0;


   @Override
    public void runOpMode(){


       // drone = hardwareMap.get(Servo.class, "drone");

        slides1 = hardwareMap.get(DcMotorEx.class, "slides1");
        slides2 = hardwareMap.get(DcMotorEx.class, "slides2");

        while (opModeInInit()){
           slides2.setDirection(DcMotorSimple.Direction.REVERSE);
            }
        while (opModeIsActive()){

          if (gamepad1.right_trigger > 0.1){
              slidesPower1 = 0.2;
              slidesPower2 = 0.2;
          }

          else if (gamepad1.left_trigger > 0.1){
                slidesPower1 = -0.2;
                slidesPower2 = -0.2;
          }
          else {
                slidesPower1 = 0.0;
                slidesPower2 = 0.0;
          }

            set2(slidesPower1, slidesPower2);
           telemetry.update();

        }
telemetry.update();
    }
    public void set2(double power1, double power2){
       slides1.setPower(power1);
       slides2.setPower(power2);
    }

}

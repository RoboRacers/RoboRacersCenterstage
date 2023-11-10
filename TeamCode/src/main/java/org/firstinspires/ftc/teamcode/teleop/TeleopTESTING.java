package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.statemachines.IntakeSM;
import org.firstinspires.ftc.teamcode.modules.statemachines.LauncherSM;
import org.firstinspires.ftc.teamcode.modules.subsystems.Intake;
import org.firstinspires.ftc.teamcode.modules.subsystems.Launcher;

@TeleOp(name = "Launcher_TEST", group = "16481-Power-Play")
public class TeleopTESTING extends LinearOpMode {

    double averagePos;

    //PID STUFF
    double integralSum = 0;
    double kp = 0;
    double ki = 0;
    double kd = 0;
    private double lasterror = 0;

//Servo drone;
DcMotorEx slides1;
DcMotorEx slides2;

Servo s1;
Servo s2;
ElapsedTime timer = new ElapsedTime();

   @Override
    public void runOpMode(){


       // drone = hardwareMap.get(Servo.class, "drone");
        slides1 = hardwareMap.get(DcMotorEx.class, "slides1");
        slides2 = hardwareMap.get(DcMotorEx.class, "slides2");
        slides2.setDirection(DcMotorSimple.Direction.REVERSE);
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s2.setDirection(Servo.Direction.REVERSE);

        while (opModeInInit()){
            set1(0.25);
            }
        while (opModeIsActive()){
            set1(0);
//            if (gamepad1.dpad_up) {
       //     slidesSet(0.2);
//            }
//            if (gamepad1.dpad_down) {
//                slidesSet(-0.2);
//            }
//            else {
//                slidesSet(0.0);
//            }
            /*
            if (gamepad1.a){
                drone.setPosition(0.25);
                telemetry.addData("Drone is being launched", "");
            }
            else if (gamepad1.b){
                drone.setPosition(0.00);
                telemetry.addData("Drone is not being launched", "");
            }
             */

        }
telemetry.update();
    }
    public void set1(double pos1){
        s1.setPosition(pos1);
        s2.setPosition(pos1);
    }





public void slidesSet(double slidesPower){
       slides1.setPower(slidesPower);
       slides2.setPower(slidesPower);
}


    /*
    public void setLiftPosition(int pos) {
        slides.setTargetPosition(pos);
    }
    public double PIDcontrol(double reference, double state){
       double error = reference - state;
       integralSum += error * timer.seconds();
       double derivative = (error-lasterror) / timer.seconds();
       lasterror = error;
       timer.reset();
       double output = (error*kp) + (derivative *kd) + (integralSum*ki);
               return output;
    }
    */
}
            // Telemetry
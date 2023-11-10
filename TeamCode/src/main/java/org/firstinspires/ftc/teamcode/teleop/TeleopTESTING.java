package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
DcMotorEx slides;
ElapsedTime timer = new ElapsedTime();

   @Override
    public void runOpMode(){

       // drone = hardwareMap.get(Servo.class, "drone");
        slides = hardwareMap.get(DcMotorEx.class, "slides");

        while (opModeInInit()){
            }
        while (opModeIsActive()){
            if (gamepad1.dpad_up) {
                slides.setPower(0.2);
            }
            if (gamepad1.dpad_down) {
                slides.setPower(-0.2);
            }
            else {
                slides.setPower(0.0);
            }
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
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

@TeleOp(name = "PID_Ball_Control", group = "16481-Centerstage")
public class PID_Ball_Control extends LinearOpMode{
    //define sensors and servo here
    //also variables
    private DistanceSensor distance_sensor_range;
    private Servo balance_servo;

    double kp = 0, ki = 0, kd = 0;
    double error;
    

    @Override
    public void runOpMode(){
        //stuff goes here hehehehehehe

        
    }


}

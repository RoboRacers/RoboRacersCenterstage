package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.statemachines.SlidesSM;
import org.firstinspires.ftc.teamcode.modules.subsystems.Slides;

import org.apache.commons.math3.analysis.function.Min;

@TeleOp(name = "Teleop No SM", group = "16481-Power-Play")
public class TeleopNoSM extends LinearOpMode {

//Servo drone;
//DcMotorEx slides1;
//DcMotorEx slides2;

    Servo Bs1;
    Servo Bs2;
    Servo Ss1;
    Servo Ss2;
    Servo Cs1;
    Servo drone;
    ElapsedTime timer = new ElapsedTime();

    double Increment = 0.001;
    double MINPOS = 0.0;
    double MAXPOS = 1.0;
    double position1 = 0.5;
    double position2 = 0.6;
    double position3 = 0.5;
    double position4 = 0.5;
    double position5 = 0.0;

    Slides slides;

    DcMotorEx motorLeft;
    DcMotorEx motorRight;
    double driveSensitivity = .5;
    double turnSensitivity = .75;
    double liftSpeed = .5;




    @Override
    public void runOpMode(){

        //   slides1 = hardwareMap.get(DcMotorEx.class, "slides1");
        //  slides2 = hardwareMap.get(DcMotorEx.class, "slides2");
        //  slides2.setDirection(DcMotorSimple.Direction.REVERSE);
        Bs1 = hardwareMap.get(Servo.class, "Bs1");
        Bs2 = hardwareMap.get(Servo.class, "Bs2");
        Ss1 = hardwareMap.get(Servo.class, "Ss1");
        Ss2 = hardwareMap.get(Servo.class, "Ss2");
        Cs1 = hardwareMap.get(Servo.class, "Cs1");

        drone = hardwareMap.get(Servo.class, "drone");

        slides = new Slides(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Motor and Servo Setup
        motorLeft = hardwareMap.get(DcMotorEx.class, "LiftLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "LiftRight");


        while (opModeInInit()){
            Bs2.setDirection(Servo.Direction.REVERSE);
            Ss2.setDirection(Servo.Direction.REVERSE);
            Cs1.setDirection(Servo.Direction.REVERSE);
            motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            motorRight.setDirection(DcMotorSimple.Direction.FORWARD);

            motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        while (opModeIsActive()){
            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y*driveSensitivity, -gamepad1.left_stick_x*driveSensitivity, -gamepad1.right_stick_x*turnSensitivity));
            drive.update();

            if(gamepad2.dpad_up){
                position1 += Increment;
                position2 += Increment;
                if (position2 >= MAXPOS){
                    position2 = MAXPOS;
                    position1 = MAXPOS - 0.1;
                }
            }
            else if(gamepad2.dpad_down){
                position1 -= Increment;
                position2 -= Increment;
                if (position1 <= MINPOS){
                    position1 = MINPOS;
                    position2 = MINPOS + 0.1;
                }
            }
            if(gamepad2.dpad_left){
                position3 += Increment;
                position4 += Increment;
                if (position3 >= MAXPOS){
                    position3 = MAXPOS;
                    position4 = MAXPOS;
                }
            }
            else if(gamepad2.dpad_right){
                position3 -= Increment;
                position4 -= Increment;
                if (position4 <= MINPOS){
                    position3 = MINPOS;
                    position4 = MINPOS;
                }
            }
            if (gamepad2.left_bumper){
                position5 = 0.54;
            }else if(gamepad2.right_bumper){
                position5 = 0.8;
            }
            if (gamepad1.circle){
                drone.setPosition(0.56);
            }else if(gamepad1.cross){
                drone.setPosition(0.9);
            }

            if(gamepad1.dpad_up){
                slides.setPower(3);

            }else if(gamepad1.dpad_down){
                slides.setPower(-3);
            }else{
                slides.setPower(0);
            }

            // Telemetry
            telemetry.addData("Right Slide Motor", slides.rightmotor.getCurrentPosition());
            telemetry.addData("Left Slide Motor", slides.leftmotor.getCurrentPosition());

            set1(position1, position2, position3, position4, position5);
            telemetry.addData("Bs1 Value", Bs1.getPosition());
            telemetry.addData("Bs2 Value", Bs2.getPosition());
            telemetry.addData("Ss1 Value", Ss1.getPosition());
            telemetry.addData("Ss1 Value", Ss2.getPosition());
            telemetry.addData("Cs1 Value", Cs1.getPosition());

            telemetry.addData("Drone position", drone.getPosition());

            telemetry.update();

        }
        telemetry.update();
    }
    public void set1(double pos1, double pos2, double pos3, double pos4, double pos5){
        Bs1.setPosition(pos1);
        Bs2.setPosition(pos2);
        Ss1.setPosition(pos3);
        Ss2.setPosition(pos4);
        Cs1.setPosition(pos5);
    }

}
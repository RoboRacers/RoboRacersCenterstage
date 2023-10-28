package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.index.qual.NonNegative;
import org.firstinspires.ftc.teamcode.RobotCore;

@TeleOp(name = "Distance_Slow", group = "16481-Centerstage")
public class Distance_Slow extends LinearOpMode {

    RobotCore robot;
    double driveSensitivity;
    double turnSensitivity;
    Pose2d roPos;
    double speed;
    boolean change;

    public enum STATE_LOCAT{
        STATE_LOCAT_SAFEZONE,
        STATE_LOCAT_DANGER
    }

    STATE_LOCAT currentState;
    public STATE_LOCAT getState() {return currentState;}
    public STATE_LOCAT InitLOCAT = STATE_LOCAT.STATE_LOCAT_SAFEZONE;

    public void speedchange(double speedwant){
        driveSensitivity = speedwant;
        turnSensitivity = speedwant;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotCore(hardwareMap, gamepad1, gamepad2);
        speed = 0.5;
        driveSensitivity = speed;
        turnSensitivity = speed;
        change = false;

        //runs once after init

        while (opModeInInit()) { //runs continusly after the once part
            //stuff goes here hehehehehehe

        }
        //runs when start is pressed
        //runs once between loops

        while (!isStopRequested()) { //runs contiuosly after start is pressed until stop
        //also stuff goes here hehehehehehe



            roPos = robot.drive.getPoseEstimate(); //Get Position


            switch (InitLOCAT){
                case STATE_LOCAT_SAFEZONE:
                    driveSensitivity = speed;
                    turnSensitivity = speed;
                    telemetry.addData("Max Speed", "");
                    if(change){
                        InitLOCAT = STATE_LOCAT.STATE_LOCAT_DANGER;
                        speedchange(speed);
                    }else{
                        InitLOCAT = STATE_LOCAT.STATE_LOCAT_SAFEZONE;
                        speedchange(speed);
                    }
                case STATE_LOCAT_DANGER:
                    driveSensitivity = speed;
                    turnSensitivity = speed;
                    telemetry.addData("Min Speed" , "");
                    if(change != false){
                        InitLOCAT = STATE_LOCAT.STATE_LOCAT_SAFEZONE;
                        speedchange(speed);
                    }else{
                        InitLOCAT = STATE_LOCAT.STATE_LOCAT_DANGER;
                        speedchange(speed);
                    }
            }

            robot.drive.setWeightedDrivePower(new Pose2d(gamepad1.left_stick_y*driveSensitivity, -gamepad1.left_stick_x*driveSensitivity, -gamepad1.right_stick_x*turnSensitivity));


            //scrapped; change to making the speed decrease based on the distance from backdrop
            /*
            //code gradually decreases the speed
            if (roPos == roPos){ //this will change into if in range...somehow
                if (driveSensitivity > minspeed){ // if speed is greater than the miniumum
                    driveSensitivity = driveSensitivity-speedchange;//decrease the speed by the amount we want
                    turnSensitivity = turnSensitivity-speedchange;
                }else if(driveSensitivity <= minspeed){//if speed is less than or equal to miniumum
                    driveSensitivity = minspeed; //makes speed miniumum
                    turnSensitivity = minspeed;
                }
            }else{ //if outside of range...will change sometime
                if (driveSensitivity < maxspeed){//if speed is less than max
                    driveSensitivity = driveSensitivity + speedchange;//increase speed
                    turnSensitivity = turnSensitivity + speedchange;
                }else if(driveSensitivity >= maxspeed){//if sped is greaterthan or equal to max
                    driveSensitivity = maxspeed;//make speed max
                    turnSensitivity = maxspeed;
                }
            }
             */
            telemetry.update();
        }
    }

    public void inrange(Pose2d paraPos){
        double backdropPos;
        double ycord;
        double dist;

        //defining
        backdropPos = 68;
        ycord = paraPos.getX();
        double minslow = 36;
        dist = backdropPos-ycord;
        if (dist < 32){
            
        }
    }

}

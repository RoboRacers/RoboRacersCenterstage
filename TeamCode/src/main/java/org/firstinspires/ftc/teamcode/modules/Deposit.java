package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@Disabled
@TeleOp(name = "DepositStates")
public class Deposit extends LinearOpMode {
    public enum DStates{
        extend,
        retract,
        midExtend,
        idle
    }
    public double servoFullExtendo = .5;
    public double servoMidExtendo = .25;
    public double servoRetract = 0;

    public DStates DepositStates = DStates.idle;

    public Intake.IStates IntakeStates = Intake.IStates.idle;

    Servo horizontal1;

    @Override
    public void runOpMode() {

        horizontal1 = hardwareMap.get(Servo.class, "horizontal1");

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()){
            switch(DepositStates) {
                case extend:

                    setDepositPosition(horizontal1, servoFullExtendo);
                    if(gamepad1.left_trigger==1) {
                        IntakeStates = Intake.IStates.open;
                    }
                    break;
                case retract:

                    setDepositPosition(horizontal1, servoRetract);
                    if(gamepad1.left_trigger==1) {
                        IntakeStates = Intake.IStates.open;
                    }
                    break;
                case midExtend:
                    setDepositPosition(horizontal1, servoMidExtendo);
                    if(gamepad1.left_trigger==1) {
                        IntakeStates = Intake.IStates.open;
                    }
                    break;
                default:
                    break;
            }
        }
    }

    public void setDepositPosition(Servo horizontal1, double x) {
        horizontal1.setPosition(x);
    }
}

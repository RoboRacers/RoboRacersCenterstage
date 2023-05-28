package org.firstinspires.ftc.teamcode.modules;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp(name = "LiftStates")
public class Lift extends LinearOpMode {
    public enum LStates{
        ground,
        low,
        mid,
        high,
        idle
    }
    public boolean AtPos;
    public LStates LiftStates = LStates.idle;
    public Deposit.DStates DepositStates = Deposit.DStates.idle;

    public double liftHighPos = 0;
    public double liftLowPos = 0;
    public double liftMidPos = 0;
    public double liftGroundPos = 0;

    DcMotor lift1;
    DcMotor lift2;

    @Override
    public void runOpMode() {
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()){
            switch(LiftStates) {
                case ground:
                    runPIDF(lift1, lift2, 0,0,0, 0);
                    isAtPosition(liftGroundPos);
                    if(AtPos) {
                        //deposit code here
                        if(gamepad1.a) {
                            DepositStates = Deposit.DStates.extend;
                        }
                    }
                    break;
                case low:
                    runPIDF(lift1, lift2, 0,0,0, 0);
                    isAtPosition(liftLowPos);
                    if(AtPos) {
                        //deposit code here
                        if(gamepad1.a) {
                            //depositstate = extendo or wtv
                            DepositStates = Deposit.DStates.extend;
                        }
                    }
                    break;
                case mid:
                    runPIDF(lift1, lift2, 1,0,0, 0);
                    isAtPosition(liftMidPos);
                    if(AtPos) {
                        //deposit code here
                        if(gamepad1.a) {
                            //depositstate = extendo or wtv
                            DepositStates = Deposit.DStates.extend;
                        }
                    }
                    break;
                case high:
                    runPIDF(lift1, lift2, 1,1,0, 0);
                    isAtPosition(liftHighPos);
                    if(AtPos) {
                        //deposit code here
                        if(gamepad1.a) {
                            //depositstate = extendo or wtv
                            DepositStates = Deposit.DStates.extend;
                        }
                    }
                    break;
                default:
                    break;
            }
        }
    }

    public void runPIDF(DcMotor lift1, DcMotor lift2, double kp, double ki, double kd, double kf) {

    }
    public void isAtPosition(double encoderPos) {
        if((lift1.getCurrentPosition() + lift2.getCurrentPosition())/2 + 5 <=encoderPos || (lift1.getCurrentPosition() + lift2.getCurrentPosition())/2 - 5 >=encoderPos) {
            AtPos = true;
        } else {
            AtPos = false;
        }
    }
}

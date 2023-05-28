package org.firstinspires.ftc.teamcode.modules;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name = "AutoRetract")
public class AutoRetract extends LinearOpMode {
    public enum ARetract{
        autoRetract,
        idle
    }
    public ARetract AutoRetract = ARetract.idle;
    public Lift.LStates LiftStates = Lift.LStates.idle;
    public Intake.IStates IntakeStates = Intake.IStates.idle;
    public Deposit.DStates DepositStates = Deposit.DStates.idle;
    @Override
    public void runOpMode() {
        waitForStart();
        if (isStopRequested()) return;
        while(opModeIsActive()){
            switch(AutoRetract) {
                case autoRetract:
                    LiftStates = Lift.LStates.ground;
                    IntakeStates = Intake.IStates.close;
                    DepositStates = Deposit.DStates.retract;
                    break;
                default:
                    break;
            }
        }
    }
}

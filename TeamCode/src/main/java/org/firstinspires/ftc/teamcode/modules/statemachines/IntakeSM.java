package org.firstinspires.ftc.teamcode.modules.statemachines;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.modules.subsystems.Intake;

public class IntakeSM implements StateMachine {

    Intake intake;
double closed = 0.25;
double open = 0;
double out = 0.5;
double in = 0;
double extend = 0.25;
double retract = 0;
    STATE currentState;
    CLAWSTATE currentClawState;

    public enum STATE {
        FOLDED_IN,
        REACHED_OUT,
    }

    public enum CLAWSTATE {
        CLAW_OPEN,
        CLAW_CLOSE
    }

    public enum EVENT {
        GAME_START,
        EXTEND_TO_PIXEL,
        RETRACT_WITH_PIXEL,
        GRABBING_PIXEL,
        OPEN_CLAW
    }

    public IntakeSM(Intake intake) {
        this.intake = intake;
    }

    public IntakeSM.STATE getState() {
        return currentState;
    }

    public void transition(IntakeSM.EVENT event) {
        switch (event) {
            case GAME_START:
                //NOT WORKING YET
                //NOW ITS WORKING
                currentState = IntakeSM.STATE.FOLDED_IN;
                currentClawState = IntakeSM.CLAWSTATE.CLAW_OPEN;
                break;
            case EXTEND_TO_PIXEL:
                currentState = IntakeSM.STATE.REACHED_OUT;
                break;
            case RETRACT_WITH_PIXEL:
                currentState = STATE.FOLDED_IN;
                break;
            case OPEN_CLAW:
                currentClawState = CLAWSTATE.CLAW_OPEN;
                break;
            case GRABBING_PIXEL:
                currentClawState = CLAWSTATE.CLAW_CLOSE;
                break;
        }
    }
/* WHERE DO WE ADD THE EVENT SO THAT WHEN WE CLICK A BUTTON IT SWTICHES EVENTS? IN LAUNCHER.JAVA OR Launcher.SM.java??????*/
    public void update() {
        switch (currentClawState) {
            case CLAW_OPEN:
                intake.setClawPos(open);
                break;
            case CLAW_CLOSE:
                intake.setClawPos(closed);
                break;
        }
        switch (currentState) {
            case REACHED_OUT:
                intake.setIntake(true);
               // intake.setIntakePos(out, extend, 500, 5, 500, 5, DcMotorSimple.Direction.REVERSE);
               break;
            case FOLDED_IN:
                intake.setIntake(false);
                //intake.setIntakePos(in, retract, 500, 0, 500, 0, DcMotorSimple.Direction.REVERSE);
                break;
        }
    }

}
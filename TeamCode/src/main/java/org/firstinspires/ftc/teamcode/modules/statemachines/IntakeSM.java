package org.firstinspires.ftc.teamcode.modules.statemachines;

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

    public enum STATE {
        FOLDED_IN,
        REACHED_OUT,
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
                // currentState = IntakeSM.STATE.FOLDED_IN && IntakeSM.STATE.CLAW_OPEN;
                break;
            case EXTEND_TO_PIXEL:
                currentState = IntakeSM.STATE.REACHED_OUT;
                break;
            case RETRACT_WITH_PIXEL:
                currentState = STATE.FOLDED_IN;
                break;
            case OPEN_CLAW:
                currentState = STATE.CLAW_OPEN;
                break;
            case GRABBING_PIXEL:
                currentState = STATE.CLAW_CLOSE;
                break;
        }
    }
/* WHERE DO WE ADD THE EVENT SO THAT WHEN WE CLICK A BUTTON IT SWTICHES EVENTS? IN LAUNCHER.JAVA OR Launcher.SM.java??????*/
    public void update() {
        switch (currentState) {
            case REACHED_OUT:
intake.claw_flip.setPosition(out);
intake.claw_extend_one.setPosition(extend);
intake.claw_extend_two.setPosition(extend);
                intake.claw_extend_two.setDirection(Servo.Direction.REVERSE);
                break;
            case FOLDED_IN:
                intake.claw_flip.setPosition(in);
                intake.claw_extend_one.setPosition(retract);
                intake.claw_extend_two.setPosition(retract);
                intake.claw_extend_two.setDirection(Servo.Direction.REVERSE);
                break;
            case CLAW_OPEN:
                intake.claw.setPosition(open);
                break;
            case CLAW_CLOSE:
                intake.claw.setPosition(closed);
                break;
        }
    }

}
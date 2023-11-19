package org.firstinspires.ftc.teamcode.modules.statemachines;

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
    }

    public enum EVENT {
        GAME_START,
        OPEN_FOR_PIXEL,
        CLOSING_FOR_PIXEL,
        CLOSED_WITH_PIXEL,
        EXTEND_WITH_PIXEL,
        RELEASE_PIXEL

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
                break;
            case OPEN_FOR_PIXEL:
                intake.setIntake(.47, .68, .8);
                break;
            case CLOSING_FOR_PIXEL:
                intake.setIntake(.6,.75,.8);
                break;
            case CLOSED_WITH_PIXEL:
                intake.setIntake(.47,.73,.8);
                break;
            case EXTEND_WITH_PIXEL:
                intake.setIntake(.47,.3, .18);
                break;
            case RELEASE_PIXEL:
                intake.claw.setPosition(.6);
                break;
        }
    }

    public void update() {

    }

}
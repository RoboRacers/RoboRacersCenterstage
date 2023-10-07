package org.firstinspires.ftc.teamcode.modules.statemachines;

import org.firstinspires.ftc.teamcode.modules.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.modules.subsystems.Intake;

public class IntakeSM {

    Intake intake;

    public enum STATE {
        FOLDED_IN,
        REACHED_OUT,
    }

    public enum EVENT {
        GAME_START,
        DETECTED_PIXEL_TO_GRAB,
        DETECTED_BACKDROP_DROP_PIXEL,
    }

    STATE currentState;

    public IntakeSM(Intake intake) {
        this.intake = intake;
    }

    public IntakeSM.STATE getState() {
        return currentState;
    }

    public void transition(IntakeSM.EVENT event) {
        switch (event) {
            case GAME_START:

                currentState = IntakeSM.STATE.FOLDED_IN;
                break;
            case DETECTED_PIXEL_TO_GRAB:
                currentState = IntakeSM.STATE.REACHED_OUT;
                break;
            case DETECTED_BACKDROP_DROP_PIXEL:
                currentState = IntakeSM.STATE.REACHED_OUT;
                break;
        }
    }

    public void update() {
        switch (currentState) {
            case REACHED_OUT:
                break;
            case FOLDED_IN:
                break;
        }
    }

}
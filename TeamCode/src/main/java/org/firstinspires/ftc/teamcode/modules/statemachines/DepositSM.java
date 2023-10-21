package org.firstinspires.ftc.teamcode.modules.statemachines;

import org.firstinspires.ftc.teamcode.modules.subsystems.Deposit;

public class DepositSM implements StateMachine {

    Deposit deposit;

    STATE currentState;

    public enum STATE {
        FOLDED_IN,
        REACHED_OUT,
    }

    public enum EVENT {
        GAME_START,
        DETECTED_PIXEL_TO_GRAB,
        DETECTED_BACKDROP_DROP_PIXEL,
    }

    public DepositSM(Deposit deposit) {
        this.deposit = deposit;
    }

    public DepositSM.STATE getState() {
        return currentState;
    }

    public void transition(DepositSM.EVENT event) {
        switch (event) {
            case GAME_START:
                deposit.setLiftPosition(0);
                currentState = DepositSM.STATE.FOLDED_IN;
                break;
            case DETECTED_PIXEL_TO_GRAB:
                currentState = DepositSM.STATE.REACHED_OUT;
                break;
            case DETECTED_BACKDROP_DROP_PIXEL:
                currentState = DepositSM.STATE.REACHED_OUT;
                break;
        }
    }

    public void update() {
        switch (currentState) {
            case FOLDED_IN:
                break;
            case REACHED_OUT:
                break;
        }
    }

}
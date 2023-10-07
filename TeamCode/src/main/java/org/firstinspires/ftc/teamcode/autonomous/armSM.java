package org.firstinspires.ftc.teamcode.autonomous;

public class armSM {

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

    public armSM.STATE getState() {
        return currentState;
    }

    public void transition(armSM.EVENT event) {
        switch (event) {
            case GAME_START:
                currentState = armSM.STATE.FOLDED_IN;
                break;
            case DETECTED_PIXEL_TO_GRAB:
                currentState = armSM.STATE.REACHED_OUT;
                break;
            case DETECTED_BACKDROP_DROP_PIXEL:
                currentState = armSM.STATE.REACHED_OUT;
                break;
        }
    }

}
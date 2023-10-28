package org.firstinspires.ftc.teamcode.modules.statemachines;

import org.firstinspires.ftc.teamcode.modules.subsystems.Slides;

public class SlidesSM implements StateMachine {

    Slides slides;

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

    public SlidesSM(Slides slides) {
        this.slides = slides;
    }

    public SlidesSM.STATE getState() {
        return currentState;
    }

    public void transition(SlidesSM.EVENT event) {
        switch (event) {
            case GAME_START:
                slides.setLiftPosition(0);
                currentState = SlidesSM.STATE.FOLDED_IN;
                break;
            case DETECTED_PIXEL_TO_GRAB:
                currentState = SlidesSM.STATE.REACHED_OUT;
                break;
            case DETECTED_BACKDROP_DROP_PIXEL:
                currentState = SlidesSM.STATE.REACHED_OUT;
                break;
        }
    }

    public void update() {
        switch (currentState) {
            case FOLDED_IN:
                slides.PIDLift(0);
                break;
            case REACHED_OUT:
                slides.PIDLift(0);//replace 0 with maxium value
                break;
        }
    }

}
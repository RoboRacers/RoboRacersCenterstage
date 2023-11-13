package org.firstinspires.ftc.teamcode.modules.statemachines;

import android.transition.Slide;

import org.firstinspires.ftc.teamcode.modules.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.teleop.SLIDES;

/**
 * Class that outlines an example state machine.
 */
public class SlidesSM implements StateMachine {

    /**
     * This is the subsystem object that this statemacine controls.
     */
    SlidesSubsystem slides;

    /**
     * The current state of this subsystem.
     */
    SLIDESTATE currentState;


    /**
     * Enum declaring all the states of this state machine.
     */
    public enum SLIDESTATE {
        REACHED_OUT,
        FOLDED_IN,
        RESTING
    }

    /**
     * Enum declaring all the events of this state machine.
     */
    public enum EVENT {
        REACH_BUTTON_PRESSED,
        FOLD_BUTTON_PRESSED,
        REACHED_TOP,
        AT_BOTTOM,
    }

    /**
     * The constructor function for this class.
     * Takes in the subsystem object that this state machine controls.
     * @param
     */
    public SlidesSM(SlidesSubsystem slides) {
        this.slides = slides;
    }

    /**
     * Returns the current state of this state machine.
     * @return
     */
    public SlidesSM.SLIDESTATE getState() {
        return currentState;
    }

    /**
     * Transition the state into a different state given an event.
     * Run one time actions in here as well.
     * @param event
     */
    public void transition(SlidesSM.EVENT event) {
        switch (event) {
            case REACH_BUTTON_PRESSED:
                // You can run one time events here.
                currentState = SLIDESTATE.REACHED_OUT;
                break;
            case FOLD_BUTTON_PRESSED:
                currentState = SLIDESTATE.FOLDED_IN;
                break;
            case AT_BOTTOM:
                currentState = SLIDESTATE.RESTING;
                break;
            case REACHED_TOP:
                currentState = SLIDESTATE.RESTING;
                break;
        }
    }

    /**
     * Updates the state every single loop. Run repetitive actions here.
     * You can define transitions to other states internally here as well.
     */
    public void update() {
        switch (currentState) {
            case REACHED_OUT:
                slides.setTargetPower(0.2);
                break;
            case FOLDED_IN:
                slides.setTargetPower(-0.2);
                break;
            case RESTING:
                slides.setTargetPower(0.0);
        }
    }

}
package org.firstinspires.ftc.teamcode.modules.statemachines;

import org.firstinspires.ftc.teamcode.modules.subsystems.Slides;

/**
 * Class that outlines an example state machine.
 */
public class SlidesSM implements StateMachine {

    /**
     * This is the subsystem object that this statemacine controls.
     */
    Slides slides;

    /**
     * The current state of this subsystem.
     */
    SLIDESTATE currentState = SLIDESTATE.MANUAL_CONTROL;


    /**
     * Enum declaring all the states of this state machine.
     */
    public enum SLIDESTATE {
        MANUAL_CONTROL,
        RUN_WITH_PID
    }

    /**
     * Enum declaring all the events of this state machine.
     */
    public enum EVENT {
        ENABLE_MANUAL,
        ENABLE_PID
    }

    /**
     * The constructor function for this class.
     * Takes in the subsystem object that this state machine controls.
     * @param
     */
    public SlidesSM(Slides slides) {
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
            case ENABLE_MANUAL:
                currentState = SLIDESTATE.MANUAL_CONTROL;
            case ENABLE_PID:
                currentState = SLIDESTATE.RUN_WITH_PID;
                break;
        }
    }

    /**
     * Updates the state every single loop. Run repetitive actions here.
     * You can define transitions to other states internally here as well.
     */
    public void update() {
        switch (currentState) {
            case RUN_WITH_PID:
                slides.setPIDPower();
                break;
            default:
                break;
        }
    }

}
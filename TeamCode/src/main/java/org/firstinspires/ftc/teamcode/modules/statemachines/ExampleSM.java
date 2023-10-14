package org.firstinspires.ftc.teamcode.modules.statemachines;

import org.firstinspires.ftc.teamcode.modules.subsystems.ExampleSubsystem;

/**
 * Class that outlines an example state machine.
 */
public class ExampleSM implements StateMachine {

    /**
     * This is the subsystem object that this statemacine controls.
     */
    ExampleSubsystem exampleSubsystem;

    /**
     * The current state of this subsystem.
     */
    STATE currentState;


    /**
     * Enum declaring all the states of this state machine.
     */
    public enum STATE {
        STATE_1,
        STATE_2
    }

    /**
     * Enum declaring all the events of this state machine.
     */
    public enum EVENT {
        EVENT_1,
        EVENT_2,
        EVENT_3
    }

    /**
     * The constructor function for this class.
     * Takes in the subsystem object that this state machine controls.
     * @param exampleSubsystem
     */
    public ExampleSM(ExampleSubsystem exampleSubsystem) {
        this.exampleSubsystem = exampleSubsystem;
    }

    /**
     * Returns the current state of this state machine.
     * @return
     */
    public ExampleSM.STATE getState() {
        return currentState;
    }

    /**
     * Transition the state into a different state given an event.
     * Run one time actions in here as well.
     * @param event
     */
    public void transition(ExampleSM.EVENT event) {
        switch (event) {
            case EVENT_1:
                // You can run one time events here.
                currentState = STATE.STATE_1;
                break;
            case EVENT_2:

                currentState = STATE.STATE_2;
                break;
            case EVENT_3:

                currentState = STATE.STATE_2;
                break;
        }
    }

    /**
     * Updates the state every single loop. Run repetitive actions here.
     * You can define transitions to other states internally here as well.
     */
    public void update() {
        switch (currentState) {
            case STATE_1:
                // Put this actions here.
                break;
            case STATE_2:
                break;
        }
    }

}
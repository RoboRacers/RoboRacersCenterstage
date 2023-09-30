package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.autonomous.LiftStateMachineTest.event;
import org.firstinspires.ftc.teamcode.autonomous.LiftStateMachineTest.state;

public class LiftStateMachineTest {

    private enum state{
        DRIVING_TOWARDS_A_SPIKE_MARKER,
        ROBOT_REACHED_THE_SPIKE_MARKER,
        DRIVING_TOWARDS_THE_BACK_BACKDROP,
        REACHED_THE_BACKDROP,
        DRIVING_TOWARDS_THE_STACK_ONE,
        REACHED_STACK_ONE,
        DRIVING_TOWARDS_STACK_TWO,
        REACHED_STACK_TWO,
        DRIVE_TOWARDS_BACKDROP,
        LOOP,
    }

    private enum event{
        DETECTED_THE_TEAM_PROP,
        LOCATION_XYZ,
        DROPPED_PURPLE_PIXEL,
        LOCATION,
        DROP_YELLOW_PIXEL,
        LOCATION_STACK_ONE,
        PIXEL_IS_IN_CLAW,
        LOCATION_STACK_TWO,
        TWO_PIXELS_IN_CLAW,
        LOOP,
    }
    public void transition(event) {
        switch (event) {
            case event.DETECTED_THE_TEAM_PROP:
                currentState = state.DRIVING_TOWARDS_A_SPIKE_MARKER;
                break;
            case event.LOCATION_XYZ:
                currentState = state.ROBOT_REACHED_THE_SPIKE_MARKER;
                break;
            case event.DROPPED_PURPLE_PIXEL:
                currentState = state.DRIVING_TOWARDS_THE_BACK_BACKDROP;
                break;
            case event.LOCATION:
                currentState = state.REACHED_THE_BACKDROP;
                break;
            case event.DROP_YELLOW_PIXEL:
                currentState = state.DRIVING_TOWARDS_THE_STACK_ONE;
                break;
            case event.LOCATION_STACK_ONE:
                currentState = state.REACHED_STACK_ONE;
                break;
            case event.PIXEL_IS_IN_CLAW:
                currentState = state.DRIVING_TOWARDS_STACK_TWO;
                break;
            case event.LOCATION_STACK_TWO:
                currentState = state.REACHED_STACK_TWO;
                break;
            case event.TWO_PIXELS_IN_CLAW:
                currentState = state.DRIVE_TOWARDS_BACKDROP;
                break;
            case event.LOOP:
                currentState = state.LOOP;
                break;
        }
        public void action(state) {
            switch (state) {
                case state.DRIVING_TOWARDS_A_SPIKE_MARKER:
                    //We must put the drive function to drive toward the spike marker with the team prop
                    break;
                case state.ROBOT_REACHED_THE_SPIKE_MARKER:
                    //We must put the drive function that tells us when we reach the spike marker

                    break;
                case state.DRIVING_TOWARDS_THE_BACK_BACKDROP:
                    //We must put the drive function that drives toward the back drop
                    break;

                case state.REACHED_THE_BACKDROP:
                    //We must put the drive function that tells us when we reach the back drop
                    break;
                case state.DRIVING_TOWARDS_THE_STACK_ONE:
                    //We must put the drive function to go to stack 1
                    break;
                case state.REACHED_STACK_ONE:
                    //We must put the drive function to tell us that we reached stack 1
                    break;

                case state.DRIVING_TOWARDS_STACK_TWO:
                    //We must put the drive function to drive to stack 2
                    break;
                case state.REACHED_STACK_TWO:
                    //We must put the drive function to tell us once we reach stack 2
                    break;
                case state.DRIVE_TOWARDS_BACKDROP:
                    //We must put the drive function to drive to the backdrop
                    break;

                case state.LOOP:
                    //We must put the drive function to loop the code for the stacks
                    break;
            }
    }
}
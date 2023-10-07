package org.firstinspires.ftc.teamcode.autonomous;

public class driveSM {

    public enum STATE{
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

    public enum EVENT{
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


    STATE currentState;

    public STATE getState(){
        return currentState;
    }
    public void transition( EVENT event) {
        switch (event) {
            case DETECTED_THE_TEAM_PROP:
                currentState = STATE.DRIVING_TOWARDS_A_SPIKE_MARKER;
                break;
            case LOCATION_XYZ:
                currentState = STATE.ROBOT_REACHED_THE_SPIKE_MARKER;
                break;
            case DROPPED_PURPLE_PIXEL:
                currentState = STATE.DRIVING_TOWARDS_THE_BACK_BACKDROP;
                break;
            case LOCATION:
                currentState = STATE.REACHED_THE_BACKDROP;
                break;
            case DROP_YELLOW_PIXEL:
                currentState = STATE.DRIVING_TOWARDS_THE_STACK_ONE;
                break;
            case LOCATION_STACK_ONE:
                currentState = STATE.REACHED_STACK_ONE;
                break;
            case PIXEL_IS_IN_CLAW:
                currentState = STATE.DRIVING_TOWARDS_STACK_TWO;
                break;
            case LOCATION_STACK_TWO:
                currentState = STATE.REACHED_STACK_TWO;
                break;
            case TWO_PIXELS_IN_CLAW:
                currentState = STATE.DRIVE_TOWARDS_BACKDROP;
                break;
            case LOOP:
                currentState = STATE.LOOP;
                break;
        }

    }

    public void update() {
        switch (currentState) {
             
        }

    }
}
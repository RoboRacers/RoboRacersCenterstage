package org.firstinspires.ftc.teamcode.modules.statemachines;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.subsystems.Intake;

import java.util.concurrent.TimeUnit;

public class IntakeSM implements StateMachine {

    Intake intake;
    STATE currentState = STATE.EMPTY_INTAKE;

    /**
     * Stores any events that are queued to be executed but were blocked.
     */
    EVENT pendingEvent;

    ElapsedTime timer;



    public enum STATE {
        INTAKING,
        LOADED_INTAKE,
        EMPTY_INTAKE,
        LOADED_DEPOSIT,
        EMPTY_DEPOSIT,
        TRANSITIONING

    }

    public enum EVENT {
        START_INTAKE,
        PIXELS_LOADED,
        // Note: This event is used only as an emergency stop for the intake
        KILL_INTAKE,
        FLIP_INTAKE,
        FLIP_DEPOSIT,

    }

    public IntakeSM(Intake intake) {
        this.intake = intake;
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public IntakeSM.STATE getState() {
        return currentState;
    }

    public void transition(IntakeSM.EVENT event) {
        switch (event) {
            case FLIP_DEPOSIT:
                if (intake.PIXELS_LOCKED) intake.flipDeposit();
                currentState = STATE.LOADED_DEPOSIT;
                break;
            case FLIP_INTAKE:
                intake.flipIntake();
                currentState = STATE.EMPTY_INTAKE;
            default:
                break;
        }
    }

    public void update() {
        switch (currentState) {
            case LOADED_INTAKE:
                break;
            case EMPTY_DEPOSIT:
                break;
            case EMPTY_INTAKE:
                break;
            case TRANSITIONING:
                break;
        }
    }

}
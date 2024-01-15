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
        KILL_INTAKE

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
            case START_INTAKE:
                if (currentState == STATE.EMPTY_INTAKE) {
                    intake.clearLock(true, true);
                    // Reset the timer that tracks how long the intake has been running
                    timer.reset();
                    currentState = STATE.INTAKING;
                } else if (currentState == STATE.EMPTY_DEPOSIT) {
                    intake.flipIntake();
                    currentState = STATE.TRANSITIONING;
                    // Set the event as pending to run later
                    pendingEvent = EVENT.START_INTAKE;
                }
            case PIXELS_LOADED:
                intake.engageLock(true, true);
                timer.reset();
                currentState = STATE.LOADED_INTAKE;
            case KILL_INTAKE:
                if (currentState == STATE.INTAKING) {
                    intake.setIntakePower(0);
                    currentState = STATE.EMPTY_INTAKE;
                }
            default:
                break;
        }
    }

    public void update() {
        switch (currentState) {
            case INTAKING:
                // Spin the rolling intake
                intake.setIntakePower(0.6);
                // If intaking has timed out, switch
                if (timer.now(TimeUnit.SECONDS) > 5) {
                    this.transition(
                            EVENT.PIXELS_LOADED
                    );
                }
                break;
            case LOADED_INTAKE:
                // Reverse the intake for the first part of this state
                if (timer.now(TimeUnit.SECONDS) < 1.25) {
                    intake.setIntakePower(-0.6);
                } else {
                    intake.setIntakePower(0);
                }
                break;
            case EMPTY_DEPOSIT:
                break;
            case EMPTY_INTAKE:

                // If the intake start has been pending, start it now
                if (pendingEvent == EVENT.START_INTAKE) {
                    this.transition(
                            EVENT.START_INTAKE
                    );
                    pendingEvent = null;
                }
                break;
            case TRANSITIONING:
                // If the deposit is in the deposit position
                /*
                if (intake.inDepositPosition()) {
                    currentState = STATE.EMPTY_DEPOSIT;
                }
                if (intake.inIntakePosition()) {
                    currentState = STATE.EMPTY_INTAKE;
                }
                 */
                break;
        }
    }

}
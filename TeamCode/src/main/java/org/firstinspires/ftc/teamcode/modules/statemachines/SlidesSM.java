package org.firstinspires.ftc.teamcode.modules.statemachines;

import com.qualcomm.robotcore.hardware.DcMotor;

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
    STATE currentState = STATE.MANUAL_CONTROL;


    /**
     * Enum declaring all the states of this state machine.
     */
    public enum STATE {
        MANUAL_CONTROL,
        RUN_WITH_PID,
        RUN_TO_POSITION
    }

    /**
     * Enum declaring all the events of this state machine.
     */
    public enum EVENT {
        ENABLE_MANUAL,
        ENABLE_PID,
        ENABLE_RTP
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
    public SlidesSM.STATE getState() {
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
                slides.leftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slides.rightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                currentState = STATE.MANUAL_CONTROL;
            case ENABLE_PID:
                slides.leftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slides.rightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //currentState = SLIDESTATE.RUN_WITH_PID;
                break;
            case ENABLE_RTP:
                slides.leftmotor.setTargetPosition(slides.getTargetPosition());
                slides.rightmotor.setTargetPosition(slides.getTargetPosition());
                slides.leftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slides.rightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                currentState = STATE.RUN_TO_POSITION;
        }
    }

    /**
     * Updates the state every single loop. Run repetitive actions here.
     * You can define transitions to other states internally here as well.
     */
    public void update() {
        switch (currentState) {
            case RUN_WITH_PID:
                //slides.setPIDPower();
                break;
            case RUN_TO_POSITION:
                slides.rightmotor.setTargetPosition(slides.getTargetPosition());
                slides.leftmotor.setTargetPosition(slides.getTargetPosition());
                break;
            default:
                break;
        }
    }

}
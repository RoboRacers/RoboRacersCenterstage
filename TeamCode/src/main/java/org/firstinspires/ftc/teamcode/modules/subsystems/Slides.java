package org.firstinspires.ftc.teamcode.modules.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.statemachines.SlidesSM;

public class Slides extends Subsystem {

    /**
     * The statemachine object associated with this subsystem.
     */
    public SlidesSM statemachine;

    /*
     * More variables/objects related to the operation of this subsystem.
     */

    public DcMotor rightmotor;
    public DcMotor leftmotor;

    private int targetPosition = 0;

    /**
     * The constructor class for this subsystem. Do all the setup
     * needed in this function related to setting up the motors, etc.
     * @param hardwareMap
     */
    public Slides(HardwareMap hardwareMap) {
        // Set up the motor related to this subsystem
        rightmotor = hardwareMap.get(DcMotor.class, "rightSlide");
        leftmotor = hardwareMap.get(DcMotor.class, "leftSlide");

        rightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Setup the state machine associated with
        statemachine = new SlidesSM(this);
    }

    /*
     * Some arbitrary functions specific to this subsystem.
     */

    /**
     * Set the target for the slies using RUN_TO_POSITION
     * @param pos
     */
    public void setTargetPosition(int pos) {
        targetPosition = pos;

        leftmotor.setTargetPosition(targetPosition);
        rightmotor.setTargetPosition(targetPosition);

        rightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPower(double power){
        rightmotor.setPower(power);
        leftmotor.setPower(power);
    }

    public void setManualPower(double power){
        rightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightmotor.setPower(power);
        leftmotor.setPower(power);
    }

    /**
     * Function that will run every single loop. Run any code that
     * needs to be run every loop here, and update the statemachine object.
     */
    @Override
    public void update() {
        statemachine.update();
    }
}

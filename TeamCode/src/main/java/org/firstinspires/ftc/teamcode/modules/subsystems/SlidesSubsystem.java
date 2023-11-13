package org.firstinspires.ftc.teamcode.modules.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.statemachines.SlidesSM;

public class SlidesSubsystem extends Subsystem {

    /**
     * The statemachine object associated with this subsystem.
     */
    public SlidesSM statemachine;

    /*
     * More variables/objects related to the operation of this subsystem.
     */

    public DcMotor rightmotor;
    public DcMotor leftmotor;

    private double targetPower = 0.0;

    /**
     * The constructor class for this subsystem. Do all the setup
     * needed in this function related to setting up the motors, etc.
     * @param hardwareMap
     */
    public SlidesSubsystem(HardwareMap hardwareMap) {
        // Set up the motor related to this subsystem
        rightmotor = hardwareMap.get(DcMotor.class, "rightSlide");
        leftmotor = hardwareMap.get(DcMotor.class, "leftSlide");

        // Setup the state machine associated with
        statemachine = new SlidesSM(this);
    }

    /*
     * Some arbitrary functions specific to this subsystem.
     */

    public void setTargetPower(double pos) {
        targetPower = pos;
    }

    public void setPower(double power){
        rightmotor.setPower(power);
        leftmotor.setPower(power);
    }

    public boolean checkPos(int target){
        int Rpos = rightmotor.getCurrentPosition();
        if (target < Rpos+1){
            if (target > Rpos-1){
                return true;
            }
        }
        return false;
    }




    /**
     * Function that will run every single loop. Run any code that
     * needs to be run every loop here, and update the statemachine object.
     */
    @Override
    public void update() {
        // Run whatever code you need to run every time here.
        setPower(targetPower);
        // Don't forget to update the state machine!

        if(checkPos(2)){ //check if its at its max position
            statemachine.transition(SlidesSM.EVENT.REACHED_TOP);
        }else if(checkPos(1)){ // check if its at its min positiion
            statemachine.transition(SlidesSM.EVENT.AT_BOTTOM);
        }

        statemachine.update();
    }
}

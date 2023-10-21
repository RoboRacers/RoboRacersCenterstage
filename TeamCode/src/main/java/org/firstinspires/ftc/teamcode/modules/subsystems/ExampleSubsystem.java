package org.firstinspires.ftc.teamcode.modules.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.statemachines.DepositSM;
import org.firstinspires.ftc.teamcode.modules.statemachines.ExampleSM;

public class ExampleSubsystem extends Subsystem {

    /**
     * The statemachine object associated with this subsystem.
     */
    public ExampleSM statemachine;

    /*
     * More variables/objects related to the operation of this subsystem.
     */

    public DcMotor testMotor;

    private int targetPosition;

    /**
     * The constructor class for this subsystem. Do all the setup
     * needed in this function related to setting up the motors, etc.
     * @param hardwareMap
     */
    public ExampleSubsystem(HardwareMap hardwareMap) {
        // Set up the motor related to this subsystem
        testMotor = hardwareMap.get(DcMotor.class, "leftLift");

        // Setup the state machine associated with
        statemachine = new ExampleSM(this);
    }

    /*
     * Some arbitrary functions specific to this subsystem.
     */

    public void setTargetPosition(int pos) {
        targetPosition = pos;
    }

    public void setMotorSpeed(double speed) {
        testMotor.setPower(speed);
    }

    /**
     * Function that will run every single loop. Run any code that
     * needs to be run every loop here, and update the statemachine object.
     */
    @Override
    public void update() {
        // Run whatever code you need to run every time here.
        testMotor.setTargetPosition(targetPosition);

        // Don't forget to update the state machine!
        statemachine.update();
    }
}

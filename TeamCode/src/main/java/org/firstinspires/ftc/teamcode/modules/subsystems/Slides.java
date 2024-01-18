package org.firstinspires.ftc.teamcode.modules.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.statemachines.SlidesSM;

public class Slides implements Subsystem {

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

    double kP = 0.01;
    double kI = 0;
    double kD = 0;

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
        leftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
    }

    public int getTargetPosition() {
        return targetPosition;
    }
    public void setPID(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }

    public int getCurrentPosition() {
        return leftmotor.getCurrentPosition();
    }

    public void setPower(double power){
        rightmotor.setPower(power);
        leftmotor.setPower(power);
    }

    public void setManualPower(double power){
        leftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightmotor.setPower(power);
        leftmotor.setPower(power);
    }

    /**
     * Sets the power based on the internal PID.
     * @param scalar Allows custom modulation of power.
     */
    public void setPIDPower(double scalar) {
        double currentPosition = leftmotor.getCurrentPosition();
        BasicPID pid = new BasicPID(new PIDCoefficients(kP, kI, kD));

        double power = pid.calculate(targetPosition, currentPosition);

        leftmotor.setPower(scalar*power);
        rightmotor.setPower(scalar*power);
    }

    /**
     * Sets the power based on the internal PID.
     */
    public void setPIDPower() {
        double currentPosition = leftmotor.getCurrentPosition();
        BasicPID pid = new BasicPID(new PIDCoefficients(kP, kI, kD));

        double power = -pid.calculate(targetPosition, currentPosition);

        leftmotor.setPower(power);
        rightmotor.setPower(power);
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

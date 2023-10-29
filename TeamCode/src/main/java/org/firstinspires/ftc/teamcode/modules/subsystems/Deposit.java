package org.firstinspires.ftc.teamcode.modules.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.statemachines.DepositSM;

import java.nio.channels.AcceptPendingException;

public class Deposit extends Subsystem {

    public DepositSM statemachine;

    public DcMotor leftMotor;
    public DcMotor rightMotor;

    double leftMotorPos;
    double rightMotorPos;

    public Deposit(HardwareMap hardwareMap) {

        leftMotor = hardwareMap.get(DcMotor.class, "leftLift");
        rightMotor = hardwareMap.get(DcMotor.class, "rightLight");

        statemachine = new DepositSM(this);
    }

    public void setLiftPosition(int pos) {
        leftMotor.setTargetPosition(pos);
        leftMotor.setTargetPosition(pos);
    }

    public final class SetLiftPostion implements Action {

        /**
         *
         * @param telemetryPacket
         * @return
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            leftMotor.setTargetPosition(5);
            leftMotor.setTargetPosition(5);

            return false; // Makes this only run once, returning true runs this again
        }
    }

    @Override
    public void update() {
        leftMotorPos = leftMotor.getCurrentPosition();
        rightMotorPos = rightMotor.getCurrentPosition();

        statemachine.update();
    }
}

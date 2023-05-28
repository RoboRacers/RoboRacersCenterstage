package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Flipbar Test")
public class FlipbarTest extends LinearOpMode {
    Servo flipbarLeft;
    Servo flipbarRight;

    final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    final int    CYCLE_MS    =   50;     // period of each cycle
    final double MAX_POS     =  1.0;     // Maximum rotational position
    final double MIN_POS     =  0.0;     // Minimum rotational position
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;

    @Override
    public void runOpMode() {
        flipbarLeft = hardwareMap.get(Servo.class, "fbl");
        flipbarRight = hardwareMap.get(Servo.class, "fbr");

        flipbarLeft.setDirection(Servo.Direction.REVERSE);
        flipbarRight.setDirection(Servo.Direction.FORWARD);

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();

        flipbarLeft.setPosition(position);
        flipbarRight.setPosition(position);

        waitForStart();

        while(opModeIsActive()){
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT ;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }

            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            flipbarLeft.setPosition(position);
            flipbarRight.setPosition(position);

            sleep(CYCLE_MS);
            idle();
        }
    }
}
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/*
 * This OpMode scans a single servo back and forward until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a Robot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "Individual Scan Servo", group = "Test")
public class IndividualServoTest extends LinearOpMode {

    static final double INCREMENT   = 0.02;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   40;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    ServoImplEx rightServo;
    ServoImplEx   leftServo;
    double  position = .6; // Start at halfway position

    double  lposition = .72; // Start at halfway position


    // L Deposit Pos : 0.96
    // L safe pos: 0.12
    // L intake pos: 0.02
    // R deposit pos: 0.88
    // R safe pos: 0.10
    // R Intake pos: 0
    boolean rampUp = true;
    boolean lrampUp = true;


    @Override
    public void runOpMode() {

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        rightServo = hardwareMap.get(ServoImplEx.class, "flipRight");
        leftServo = hardwareMap.get(ServoImplEx.class, "flipLeft");


        leftServo.setDirection(Servo.Direction.REVERSE);

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();

        boolean leftEnable = false;
        boolean rightEnable = false;

        boolean previousRBPress = false;
        boolean previousLBPress = false;


        // Scan servo till stop pressed.
        while(opModeIsActive()){


            if (gamepad1.triangle) {
                rightServo.setPwmDisable();
                // Keep stepping up until we hit the max value.
                lposition += INCREMENT ;
                if (lposition >= MAX_POS ) {
                    lposition = MAX_POS;
                    lrampUp = !lrampUp;   // Switch ramp direction
                }
                leftServo.setPosition(lposition);
            }
            else if (gamepad1.cross) {
                rightServo.setPwmDisable();
                // Keep stepping down until we hit the min value.
                lposition -= INCREMENT ;
                if (lposition <= MIN_POS ) {
                    lposition = MIN_POS;
                    lrampUp = !lrampUp;  // Switch ramp direction
                }
                leftServo.setPosition(lposition);
            }

            if (gamepad1.dpad_up) {
                leftServo.setPwmDisable();
                // Keep stepping up until we hit the max value.
                position += INCREMENT ;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    rampUp = !rampUp;   // Switch ramp direction
                }
                rightServo.setPosition(position);
            }
            else if (gamepad1.dpad_down) {
                leftServo.setPwmDisable();
                // Keep stepping down until we hit the min value.
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                }
                rightServo.setPosition(position);
            }

            if (gamepad1.right_bumper && !previousRBPress) {
                rightEnable = !rightEnable;
                if (rightEnable) {
                    rightServo.setPwmEnable();
                } else {
                    rightServo.setPwmDisable();
                }
            }

            if (gamepad1.left_bumper && !previousLBPress) {
                leftEnable = !leftEnable;
                if (leftEnable) {
                    leftServo.setPwmEnable();
                } else {
                    leftServo.setPwmDisable();
                }
            }

            // Display the current value
            telemetry.addData("Right Servo Position", "%5.2f", rightServo.getPosition());

            telemetry.addData("Left Servo Position", "%5.2f", leftServo.getPosition());
            telemetry.addData("Right Servo enable", rightServo.isPwmEnabled());
            telemetry.addData("Left Servo Enable", leftServo.isPwmEnabled());
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            sleep(CYCLE_MS);
            idle();

            previousLBPress = gamepad1.left_bumper;
            previousRBPress = gamepad1.right_bumper;
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}

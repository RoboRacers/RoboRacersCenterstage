package org.firstinspires.ftc.teamcode.autonomous;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="SM test", group="SM OpMode")
public class drive_sm_test extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        driveSM mysm = new driveSM();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.a) {
                mysm.transition(driveSM.EVENT.DETECTED_THE_TEAM_PROP);
                telemetry.addData("a pressed", "Initialized");
                telemetry.update();
            }
            else if (gamepad1.b) {
                mysm.transition(driveSM.EVENT.LOCATION_XYZ);
                telemetry.addData("b pressed", "Initialized");
                telemetry.update();
            }
            else if (gamepad1.x) {
                mysm.transition(driveSM.EVENT.DROPPED_PURPLE_PIXEL);
                telemetry.addData("x pressed", "Initialized");
                telemetry.update();
            }
            else if (gamepad1.y) {
                mysm.transition(driveSM.EVENT.LOCATION);
                telemetry.addData("y pressed", "Initialized");
                telemetry.update();
            }
            else if (gamepad1.dpad_down) {
                mysm.transition(driveSM.EVENT.DROP_YELLOW_PIXEL);
                telemetry.addData("dpad down pressed", "Initialized");
                telemetry.update();
            }
            else if (gamepad1.dpad_up) {
                mysm.transition(driveSM.EVENT.LOCATION_STACK_ONE);
                telemetry.addData("dpad up pressed", "Initialized");
                telemetry.update();
            }
            else if (gamepad1.dpad_left) {
                mysm.transition(driveSM.EVENT.PIXEL_IS_IN_CLAW);
                telemetry.addData("dpad left pressed", "Initialized");
                telemetry.update();
            }
            else if (gamepad1.dpad_right) {
                mysm.transition(driveSM.EVENT.LOCATION_STACK_TWO);
                telemetry.addData("dpad right pressed", "Initialized");
                telemetry.update();
            }
            else if (gamepad1.right_bumper) {
                mysm.transition(driveSM.EVENT.TWO_PIXELS_IN_CLAW);
                telemetry.addData("right bumper pressed", "Initialized");
                telemetry.update();
            }
            else if (gamepad1.left_bumper) {
                mysm.transition(driveSM.EVENT.LOOP);
                telemetry.addData("left bumper pressed", "Initialized");
                telemetry.update();
            }
        }
    }
}

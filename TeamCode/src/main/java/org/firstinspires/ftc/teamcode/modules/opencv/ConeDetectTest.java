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

package org.firstinspires.ftc.teamcode.modules.opencv;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.modules.opencv.ConeDetection;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Cone Detect OpMode", group="Linear Opmode")
public class ConeDetectTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        OpenCvWebcam camera;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.
                get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // ExposureControl expo =  camera.getExposureControl();
        // expo.setExposure(160, TimeUnit.MILLISECONDS);

        ConeDetection myConeDetection = new ConeDetection(camera);

        double coneCenterX = 0.0;

        int location=-1;
        while (location==-1)
        {
            if(gamepad1.a) location=0;
            if(gamepad1.b) location=1;
            if(gamepad1.x) location=2;
            if(gamepad1.y) location=3;
        }



        telemetry.addData("Location: ", location);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        location = -1;
        if (isStopRequested()) return;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("X = ", myConeDetection.getCenter());

            telemetry.update();
        }
    }
}

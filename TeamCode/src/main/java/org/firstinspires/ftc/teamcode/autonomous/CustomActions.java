package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class CustomActions {

    public static void runBlocking(Action a, Observer observer) {
        boolean b = true;
        while (b && !Thread.currentThread().isInterrupted()) {
            if (!observer.checkForInterrupt()) {
                break;
            }
            TelemetryPacket p = new TelemetryPacket();
            b = a.run(p);
        }
    }
}

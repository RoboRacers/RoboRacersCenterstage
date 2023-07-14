package org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.drive.StandardTrackingWheelLocalizer;

/**
 * Class to store and load sensor values used by the MCL (Monte Carlo Localizer)
 */
public class SensorBuffer implements Sensors {
    /* Flags for user to change */

    static StandardTrackingWheelLocalizer trackingWheelLocalizer;
    static public Pose2d trackingWheelPose;
    static public double trackingWheelWeight = 1.1;

    static public void init(HardwareMap hardwareMap) {
        trackingWheelLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
    }

    public static void update(){
        trackingWheelLocalizer.update();
        trackingWheelPose = trackingWheelLocalizer.getPoseEstimate();

    }
}

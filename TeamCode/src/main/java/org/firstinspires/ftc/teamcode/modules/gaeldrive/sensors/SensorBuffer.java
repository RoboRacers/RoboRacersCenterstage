package org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.drive.StandardTrackingWheelLocalizer;

/**
 * Class to store and load sensor values used by the MCL (Monte Carlo Localizer)
 */
public class SensorBuffer {
    /* Flags for user to change */
    public boolean hasTrackingWheels = true;

    StandardTrackingWheelLocalizer trackingWheelLocalizer;
    public Pose2d trackingWheelPose;
    public double trackingWheelWeight = 1.1;

    public SensorBuffer(HardwareMap hardwareMap){
        if (hasTrackingWheels){
            this.trackingWheelLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
        }
    }

    public void update(){
        if (hasTrackingWheels){
            this.trackingWheelLocalizer.update();
            this.trackingWheelPose = this.trackingWheelLocalizer.getPoseEstimate();
        }
    }
}

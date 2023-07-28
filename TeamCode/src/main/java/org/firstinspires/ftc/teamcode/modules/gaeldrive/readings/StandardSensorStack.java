package org.firstinspires.ftc.teamcode.modules.gaeldrive.readings;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.roboracers.gaeldrive.motion.MotionModel;
import com.roboracers.gaeldrive.readings.SensorStack;

import com.roboracers.gaeldrive.LocalizationConstants;

import org.firstinspires.ftc.teamcode.modules.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.motion.TrackingWheelMotionModel;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors.DistanceSensorModel;
import com.roboracers.gaeldrive.sensors.SensorModel;
import com.roboracers.gaeldrive.utils.Updatable;

import java.util.ArrayList;
import java.util.List;

/**
 * Global buffer to store and load sensor values.
 */
public class StandardSensorStack implements SensorStack {

    public static MotionModel trackingWheelMotionModel;
    static List<SensorModel> sensorModels = new ArrayList<>();

    static public void init(HardwareMap hardwareMap) {
        trackingWheelMotionModel = new TrackingWheelMotionModel(LocalizationConstants.START_POSE, new StandardTrackingWheelLocalizer(hardwareMap));

        // Config our Distance Sensor
        DistanceSensorModel distanceSensorModel = new DistanceSensorModel(
                hardwareMap.get(DistanceSensor.class, "range"),
                7.87402, 301.181,
                new Pose2d(10,5, Math.toRadians(90)));
        sensorModels.add(distanceSensorModel);
    }

    /**
     * Run one update on all sensor models
     */
    public static void update(){
        trackingWheelMotionModel.update();
        // Update all sensor models
        for (Updatable model: sensorModels) {
            model.update();
        }
    }

    /**
     * Get a list of sensor models
     * @return
     */
    public static List<SensorModel> getSensorModels() {
       return sensorModels;
    }

}

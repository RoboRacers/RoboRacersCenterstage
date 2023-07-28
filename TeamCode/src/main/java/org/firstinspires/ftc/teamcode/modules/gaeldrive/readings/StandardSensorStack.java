package org.firstinspires.ftc.teamcode.modules.gaeldrive.readings;

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

        SensorModel distanceSensorModel = new DistanceSensorModel(hardwareMap.get(DistanceSensor.class, "range"));
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

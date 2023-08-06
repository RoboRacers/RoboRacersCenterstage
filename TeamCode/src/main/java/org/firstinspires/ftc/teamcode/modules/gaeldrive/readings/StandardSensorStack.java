package org.firstinspires.ftc.teamcode.modules.gaeldrive.readings;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.roboracers.gaeldrive.motion.MotionModel;
import com.roboracers.gaeldrive.readings.SensorStack;

import com.roboracers.gaeldrive.LocalizationConstants;

import org.firstinspires.ftc.teamcode.modules.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.motion.TrackingWheelMotionModel;

import com.roboracers.gaeldrive.sensors.SensorModel;
import com.roboracers.gaeldrive.utils.Updatable;

import java.util.ArrayList;
import java.util.List;

/**
 * Global buffer to store and load sensor values.
 */
public class StandardSensorStack implements SensorStack {

    public static MotionModel motionModel;
    static List<SensorModel> sensorModels = new ArrayList<>();

    /**
     * Initialize the Sensor stack. Initialize the common sensors and motion models here.
     * TODO: Update this with your own sensors.
     * @param hardwareMap
     */
    static public void init(HardwareMap hardwareMap) {
        motionModel = new TrackingWheelMotionModel(LocalizationConstants.START_POSE, new StandardTrackingWheelLocalizer(hardwareMap));

        // Config our Ultrasonic Distance Sensor
        //AnalogDistanceSensorModel distanceSensorModel = SensorUtils.createMB1240Sensor(hardwareMap.get(AnalogInput.class,"ultrasonic1"), new Pose2d(0,0, 0));
        //sensorModels.add(distanceSensorModel);
    }

    /**
     * Run one update on all sensor models
     */
    public static void update(){
        // Update our motion model
        motionModel.update();
        // Update all sensor models
        for (Updatable model: sensorModels) {
            model.update();
        }
    }

    /**
     * Get a list of sensor models in the sensor stack. Call update to get the latest data.
     * @return sensorModels List of sensor models
     */
    public static List<SensorModel> getSensorModels() {
       return sensorModels;
    }

    /**
     * Add a sensor model to the stack after initialization.
     * @param model The model that is needed to be added
     */
    public static void addSensorModel(SensorModel model) {
        sensorModels.add(model);
    }

}

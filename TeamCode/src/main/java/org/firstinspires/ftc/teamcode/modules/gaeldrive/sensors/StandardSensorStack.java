package org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.roboracers.gaeldrive.sensors.SensorStack;

import org.firstinspires.ftc.teamcode.modules.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.LocalizationConstants;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.motion.TrackingWheelMotionModel;

/**
 * Global buffer to store and load sensor values.
 */
public class StandardSensorStack extends SensorStack {


    /**
     * Initialize the Sensor stack. Initialize the common sensors and motion models here.
     * TODO: Update this with your own sensors.
     * @param hardwareMap
     */
    static public void init(HardwareMap hardwareMap) {
        motionModel = new TrackingWheelMotionModel(LocalizationConstants.START_POSE, new StandardTrackingWheelLocalizer(hardwareMap));

        // Config our Ultrasonic Distance Sensor
        AnalogDistanceSensorModel ultrasonicRight = SensorUtils.createMB1240Sensor(hardwareMap.get(AnalogInput.class,"ultrasonicRight"), 1, new Pose2d(-7,0, -90));
        sensorModels.add(ultrasonicRight);
        AnalogDistanceSensorModel ultrasonicLeft = SensorUtils.createMB1240Sensor(hardwareMap.get(AnalogInput.class,"ultrasonicLeft"), 1, new Pose2d(7,0, 90));
        sensorModels.add(ultrasonicLeft);
        AnalogDistanceSensorModel ultrasonicBack = SensorUtils.createMB1240Sensor(hardwareMap.get(AnalogInput.class,"ultrasonicBack"), 1, new Pose2d(-7,0, 180));
        sensorModels.add(ultrasonicBack);
    }

}

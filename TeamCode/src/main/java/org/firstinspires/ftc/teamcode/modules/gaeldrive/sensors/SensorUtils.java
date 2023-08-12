package org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;

/**
 * Util class with many pre-loaded and pre-tuned sensors for use.
 */
public class SensorUtils {

    /**
     * Creates a MB1240 with in-house tuned parameters. Change to your liking.
     * @param sensor Hardware object of sensor
     * @param weight THe weight of the sensor
     * @param sensorPose Location of the sensor on the robot. See: <a href="https://cdn.statically.io/gh/NoahBres/LearnRoadRunner/1c0fe8d5/docs/assets/dead-wheels/andrew-bot-wheel-location-quarter.jpg">LearnRoadrunner.com</a> for coordinate system
     * @return MB1240 Sensor Model
     */
    public static AnalogDistanceSensorModel createMB1240Sensor(AnalogInput sensor, double weight, Pose2d sensorPose) {
         return new AnalogDistanceSensorModel(sensor, weight,
                 7.87402, 301.181, 217.5499508, // creates a sensor with the following parameters
                 sensorPose);
    }

    /**
     * Create a a Rev 2M Distance sensor.
     * Note: Max distance is set to 150 cm despite the Rev stating that max distance is 200 cm.
     * Testing has shown that this sensor dramatically looses accuracy much before the 200 cm mark.
     * @param sensor Hardware object of sensor
     * @param weight The weight of the sensor
     * @param sensorPose Location of the sensor on the robot. See: <a href="https://cdn.statically.io/gh/NoahBres/LearnRoadRunner/1c0fe8d5/docs/assets/dead-wheels/andrew-bot-wheel-location-quarter.jpg">LearnRoadrunner.com</a> for coordinate system
     * @return Rev 2M Distance Sensor Model
     */
    public static I2CDistanceSensorModel createRev2mDistanceSensor(DistanceSensor sensor, double weight, Pose2d sensorPose) {
        return new I2CDistanceSensorModel(sensor, weight, 1.9685, 59.0551, sensorPose);
    }
}

package org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class SensorUtils {

    /**
     * Creates a MB1240 with in-house tuned parameters. Change to your liking.
     * @param sensor Hardware object of sensor
     * @param sensorPose Location of the sensor on the robot. See: <a href="https://cdn.statically.io/gh/NoahBres/LearnRoadRunner/1c0fe8d5/docs/assets/dead-wheels/andrew-bot-wheel-location-quarter.jpg">LearnRoadrunner.com</a> for coordinate system
     * @return
     */
    public static AnalogDistanceSensorModel createMB1240Sensor(AnalogInput sensor, Pose2d sensorPose) {
         return new AnalogDistanceSensorModel(sensor,
                 7.87402, 301.181, 217.5499508, // creates a sensor with the following parameters
                 sensorPose);
    }
}

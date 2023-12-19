package org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.roboracers.gaeldrive.sensors.DistanceSensorModel;

import org.firstinspires.ftc.teamcode.modules.gaeldrive.PoseUtils;

/**
 * Base class for an Analog Distance Sensor.
 */
public class AnalogDistanceSensorModel extends DistanceSensorModel {

    AnalogInput sensor;
    double conversionFactor;

    /**
     * Constructor for a distance sensor.
     * @param sensor The sensor
     * @param minDistance The minimum reading distance for this sensor
     * @param maxDistance The maximum reading distance for this sensor
     * @param location Location of the sensor on the robot. See: <a href="https://cdn.statically.io/gh/NoahBres/LearnRoadRunner/1c0fe8d5/docs/assets/dead-wheels/andrew-bot-wheel-location-quarter.jpg">LearnRoadrunner.com</a> for coordinate system
     */
    public AnalogDistanceSensorModel(AnalogInput sensor, double weight, double minDistance, double maxDistance, double conversionFactor, Pose2d location) {
        this.sensor = sensor;
        this.weight = weight;
        this.minDistance = minDistance;
        this.maxDistance = maxDistance;
        this.conversionFactor = conversionFactor;
        this.location = PoseUtils.poseToVector(location);
    }


    @Override
    public void update() {
        rawReading = sensor.getVoltage() * conversionFactor;
    }


}

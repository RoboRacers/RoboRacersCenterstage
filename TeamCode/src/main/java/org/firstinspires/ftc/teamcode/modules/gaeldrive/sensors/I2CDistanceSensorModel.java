package org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.PoseUtils;

/**
 * Class to process the data from a distance sensor to be suitable for MCL.
 * You can create your own models for whatever sensor, just use the SensorModel interface.
 * Use the FieldDistance class for distance calculations (Included in GD for your use).
 */
public class I2CDistanceSensorModel extends DistanceSensorModel {

    DistanceSensor sensor;

    /**
     * Constructor for a distance sensor.
     * @param sensor The sensor
     * @param minDistance The minimum reading distance for this sensor
     * @param maxDistance The maximum reading distance for this sensor
     * @param location Location of the sensor on the robot. See: <a href="https://cdn.statically.io/gh/NoahBres/LearnRoadRunner/1c0fe8d5/docs/assets/dead-wheels/andrew-bot-wheel-location-quarter.jpg">LearnRoadrunner.com</a> for coordinate system
     */
    public I2CDistanceSensorModel(DistanceSensor sensor, double weight, double minDistance, double maxDistance, Pose2d location) {
        this.sensor = sensor;
        this.weight = weight;
        this.minDistance = minDistance;
        this.maxDistance = maxDistance;
        this.location = PoseUtils.poseToVector(location);
    }

    @Override
    public void update() {
        rawReading = sensor.getDistance(DistanceUnit.INCH);
    }


}

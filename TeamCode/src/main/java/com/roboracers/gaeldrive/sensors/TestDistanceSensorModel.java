package com.roboracers.gaeldrive.sensors;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.roboracers.gaeldrive.distance.FieldDistance;
import com.roboracers.gaeldrive.sensors.SensorModel;
import com.roboracers.gaeldrive.utils.PoseUtils;
import com.roboracers.gaeldrive.utils.VectorUtils;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Class to process the data from a distance sensor to be suitable for MCL.
 * You can create your own models for whatever sensor, just use the SensorModel interface.
 * Use the FieldDistance class for distance calculations (Included in GD for your use).
 */
public class TestDistanceSensorModel extends DistanceSensorModel {

    /**
     * Constructor for a distance sensor.
     *
     *
     */
    public TestDistanceSensorModel(double dist, Pose2d location) {
        this.location = PoseUtils.poseToVector(location);
        this.rawReading = dist;
        this.minDistance = 5;
        this.maxDistance = 100;
        this.weight = 1;
    }

    @Override
    public void update() {

    }


}

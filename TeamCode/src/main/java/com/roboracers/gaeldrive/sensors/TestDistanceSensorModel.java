package com.roboracers.gaeldrive.sensors;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.roboracers.gaeldrive.distance.FieldDistance;
import com.roboracers.gaeldrive.sensors.SensorModel;
import com.roboracers.gaeldrive.utils.PoseUtils;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Class to process the data from a distance sensor to be suitable for MCL.
 * You can create your own models for whatever sensor, just use the SensorModel interface.
 * Use the FieldDistance class for distance calculations (Included in GD for your use).
 */
public class TestDistanceSensorModel implements SensorModel {

    double rawReading;

    // Config variables
    double weight = 0.7;
    public double minDistance = 10;
    public double maxDistance = 400;
    Pose2d location;
    double dist;

    /**
     * Constructor for a distance sensor.
     *
     *
     */
    public TestDistanceSensorModel(double dist, Pose2d location) {
        this.location = location;
        this.dist = dist;
    }

    /**
     * Gets the weight modifier assigned to this model.
     * @return Weight modifier
     */
    @Override
    public double getWeightModifier() {
        return weight;
    }

    /**
     * Returns a vectorized version of the reading.
     * @return
     */
    @Override
    public RealVector getActualReading() {
        RealVector reading = new ArrayRealVector(new double[] {rawReading});
        return reading;
    }

    /**
     * Returns the expected vectorized sensor reading from a particular state.
     * @param state the state of the particle
     * @return Simulated sensor value
     */
    @Override
    public RealVector getSimulatedReading(RealVector state) {
        double simulatedDistance = FieldDistance.calculateSimulatedDistance(PoseUtils.vectorToPose(state.add(PoseUtils.poseToVector(location))));
        // Limiting Readings to min and max distances
        if (simulatedDistance < minDistance) {
            simulatedDistance = minDistance;
        } else if (simulatedDistance > maxDistance) {
            simulatedDistance = maxDistance;
        }
        return new ArrayRealVector(new double[] {simulatedDistance});
    }

    /**
     * @return Degrees of freedom of the sensor model
     */
    @Override
    public int getDOF() {
        return 2;
    }

    /**
     * Gets the raw reading from the sensor.
     * @return Distance sensor reading
     */
    public double getRawReading () {
        return rawReading;
    }

    @Override
    public void update() {
        rawReading = dist;
    }


}

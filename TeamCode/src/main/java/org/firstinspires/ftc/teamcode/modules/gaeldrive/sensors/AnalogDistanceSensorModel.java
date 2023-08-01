package org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.roboracers.gaeldrive.distance.FieldDistance;
import com.roboracers.gaeldrive.sensors.SensorModel;
import com.roboracers.gaeldrive.utils.PoseUtils;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

/**
 * Base class for an Analog Distance Sensor.
 */
public class AnalogDistanceSensorModel implements SensorModel {

    AnalogInput sensor;
    double rawReading;

    // Config variables
    double weight = 0.7;
    // Conversion factor for converting from voltage to inches.
    double conversionFactor;
    // These values may not be the same as the manufacturer specified minimum and max distances. Tune them to your liking.
    public double minDistance;
    public double maxDistance;
    public Pose2d location;

    /**
     * Constructor for a distance sensor.
     * @param sensor The sensor
     * @param minDistance The minimum reading distance for this sensor
     * @param maxDistance The maximum reading distance for this sensor
     * @param location Location of the sensor on the robot. See: <a href="https://cdn.statically.io/gh/NoahBres/LearnRoadRunner/1c0fe8d5/docs/assets/dead-wheels/andrew-bot-wheel-location-quarter.jpg">LearnRoadrunner.com</a> for coordinate system
     */
    public AnalogDistanceSensorModel(AnalogInput sensor, double minDistance, double maxDistance, double conversionFactor, Pose2d location) {
        this.sensor = sensor;
        this.minDistance = minDistance;
        this.maxDistance = maxDistance;
        this.conversionFactor = conversionFactor;
        this.location = location;
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
        double simulatedDistance = FieldDistance.calculateSimulatedDistance(PoseUtils.vectorToPose(state));
        // Limiting Readings to min and max distances
        if (simulatedDistance < minDistance) {
            simulatedDistance = minDistance;
        } else if (simulatedDistance > maxDistance) {
            simulatedDistance = maxDistance;
        }
        return new ArrayRealVector(new double[] {simulatedDistance});
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
        rawReading = sensor.getVoltage() * conversionFactor;
    }


}

package org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.roboracers.gaeldrive.distance.FieldDistance;
import com.roboracers.gaeldrive.sensors.SensorModel;
import com.roboracers.gaeldrive.utils.PoseUtils;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorModel implements SensorModel {

    DistanceSensor sensor;
    double rawReading;

    double weight = 0.7;

    public DistanceSensorModel(DistanceSensor sensor) {
        this.sensor = sensor;
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
     * @param state
     * @return Simulated sensor value
     */
    @Override
    public RealVector getSimulatedReading(RealVector state) {
        RealVector simulatedReading = new ArrayRealVector(new double[] {FieldDistance.calculateSimulatedDistance(PoseUtils.vectorToPose(state))});
        return simulatedReading;
    }

    @Override
    public void update() {
        rawReading = sensor.getDistance(DistanceUnit.INCH);
    }

    /**
     * Gets the raw reading from the sensor.
     * @return Distance sensor reading
     */
    public double getRawReading () {
        return rawReading;
    }
}

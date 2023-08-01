package com.roboracers.gaeldrive.filters;


import org.apache.commons.math3.distribution.ChiSquaredDistribution;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors.SensorUtils;

import com.roboracers.gaeldrive.LocalizationConstants;
import com.roboracers.gaeldrive.particles.Particle;

import com.roboracers.gaeldrive.sensors.SensorModel;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A filter that uses Monte Carlo methods to find approximate solutions
 *  to filtering problems in a non-linear state space.
 */
public class ParticleFilter {

    /**
     * Hashmap that stores all the particles in Integer/Particle pairs.
     */
    public HashMap<Integer, Particle> Particles = new HashMap<>();

    /**
     * Distribution used in the resampling of particles. TODO: Make degreesOfFreed changeable.
     */
    public ChiSquaredDistribution distribution = new ChiSquaredDistribution(1);

    /**
     * Add a particle to the internal Hashmap.
     * @param particle
     */
    public void add(Particle particle) {
        Particles.put(particle.getId(), particle);
    }

    /**
     * Clear all particles.
     */
    public void clear() {
        Particles.clear();
    }

    /**
     * Return the Hashmap that stores the particles.
     * @return Hashmap of particles
     */
    public HashMap<Integer, Particle> getParticles() {
        return  this.Particles;
    }


    /**
     * Translate every particle
     * @param translationVector The translation vector that the other particles will be translated by.
     */
    public void translateParticles (RealVector translationVector) {

        for (Map.Entry<Integer,Particle> particle2dEntry : Particles.entrySet()) {
            Particle translatedParticle = particle2dEntry.getValue();
            translatedParticle.setState(translatedParticle.getState().add(translationVector));
            particle2dEntry.setValue(translatedParticle);
        }
    }

    /**
     * Weigh all the particles in our state space given a set of sensor models.
     * We compared the delta between the actual sensor reading and the
     * sensor value simulated for each one of our sensors.
     * @param models List of models to be used.
     */
    public void weighParticles(List<SensorModel> models) {

        double cumalativeWeightModifer = 0;

        HashMap<Integer, Double> weightNumerator = new HashMap<>();

        // Initialize our weight numerators
        for (Map.Entry<Integer, Particle> entry: Particles.entrySet()) {
            weightNumerator.put(entry.getKey(), 0.0);
        }

        double overallWeight = 0;

        for (SensorModel model: models) {

            for (Map.Entry<Integer, Particle> entry: Particles.entrySet()) {
                Particle particle = entry.getValue();

                double probSensorGivenState = getDeltaProbability(particle, model);

                double sensorLikelihood = probSensorGivenState * model.getWeightModifier();

                weightNumerator.put(entry.getKey(), weightNumerator.get(entry.getKey())+sensorLikelihood);

                overallWeight += sensorLikelihood;
            }

            // The sum of all the weight modifiers of all the sensor models
            cumalativeWeightModifer += model.getWeightModifier();

        }

        // For every particle in our state space
        for (Map.Entry<Integer, Particle> entry: Particles.entrySet()) {
            double weight = ((weightNumerator.get(entry.getKey())/cumalativeWeightModifer)*entry.getValue().getWeight())
                    /(overallWeight);
        }

    }

    public void resampleParticles() {

    }

    private double getDeltaProbability(Particle particle, SensorModel model) {
        // Get both the actual and simulated reading
        RealVector simulatedSensorValue = model.getSimulatedReading(particle.getState());
        RealVector actualSensorValue = model.getActualReading();

        if (LocalizationConstants.TESTING) {
            System.out.println("Real Sensor Value: "  + actualSensorValue);
            System.out.println("Simulated (Random) Sensor Value: " + simulatedSensorValue);
        }

        // Get the difference in the delta of our reading
        RealVector readingDelta = actualSensorValue.subtract(simulatedSensorValue);
        // Plug the normalized (Euclidean Distance) of the delta into our Chi2 distribution.
        return distribution.density(readingDelta.getNorm());
    }

    /**
     * Gets the particle with the highest weight.
     * @return Particle2d of the highest weighted particle.
     */
    public Particle getBestParticle () {

        double highestWeight = 0;
        Integer bestParticleKey = 0;

        // Loop through all weights and get highest weight
        for (Map.Entry<Integer, Particle> particle2dEntry : Particles.entrySet()) {
            double particleWeight = particle2dEntry.getValue().getWeight();
            if (particleWeight > highestWeight) {
                bestParticleKey = particle2dEntry.getKey();
                highestWeight = particleWeight;
            }

        }

        return Particles.get(bestParticleKey);

    }
}

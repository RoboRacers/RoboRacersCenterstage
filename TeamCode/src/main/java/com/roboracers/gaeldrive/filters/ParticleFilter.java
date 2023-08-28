package com.roboracers.gaeldrive.filters;


import com.roboracers.gaeldrive.particles.Particle;
import com.roboracers.gaeldrive.sensors.SensorModel;
import com.roboracers.gaeldrive.tests.TestConstants;

import org.apache.commons.math3.distribution.ChiSquaredDistribution;
import org.apache.commons.math3.linear.RealVector;

import java.util.ArrayList;
import java.util.List;

/**
 * A filter that uses Monte Carlo methods to find approximate solutions
 * to filtering problems in a non-linear state space.
 */
public abstract class ParticleFilter {

    /**
     * Hashmap that stores all the particles in Integer/Particle pairs.
     */
    ArrayList<Particle> Particles = new ArrayList<>();

    ChiSquaredDistribution distribution2DOF = new ChiSquaredDistribution(2);
    ChiSquaredDistribution distribution3DOF = new ChiSquaredDistribution(3);

    /**
     * Add a particle to the internal Hashmap.
     * @param particle
     */
    public void add(Particle particle) {
        Particles.add(particle);
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
    public ArrayList<Particle> getParticles() {
        return this.Particles;
    }


    /**
     * Translate every particle
     * @param translationVector The translation vector that the other particles will be translated by.
     */
    public void translateParticles (RealVector translationVector) {

        int index = 0;
        // For every particle in our set of Particles
        for (Particle particle: Particles) {
            // Add our translational vector
            particle.setState(particle.getState().add(translationVector));
            // Set the value as our updated particle
            Particles.set(index, particle);
            index ++;
        }
    }

    /**
     * Weigh all the particles in our state space given a set of sensor models.
     * We compared the delta between the actual sensor reading and the
     * sensor value simulated for each one of our sensors.
     * @param models List of models to be used.
     */
    public void weighParticles(List<SensorModel> models) {


        // For every particle in our state space
        int index = 0;
        for (Particle particle: Particles) {

            double cumalativeWeight = 0;
            double cumalativeWeightModifer = 0;

            // For every sensor model that we are considering
            for (SensorModel model: models) {

                // Get both the actual and simulated reading
                RealVector simulatedSensorValue = model.getSimulatedReading(particle.getState());
                RealVector actualSensorValue = model.getActualReading();

                double probability = readingDeltaProbability(actualSensorValue, simulatedSensorValue, model.getDOF());

                // Add the probability multiplied by the weight of the model.
                cumalativeWeight += probability * model.getWeightModifier();
                // Add the weight of this sensor model to the overall weight modifier
                cumalativeWeightModifer += model.getWeightModifier();

            }

            // Calculate the average weights of all the sensors and assign it to the particle
            particle.setWeight(cumalativeWeight/cumalativeWeightModifer);

            // Add the particle with the updated weight back into our particle set.
            Particles.set(index, particle);
            index ++;
        }
    }

    /**
     *
     * @param models List of models to be used.
     */
    public void weighParticlesBayesian(List<SensorModel> models) {
        //TODO: Implement Fully Bayesian Weighting

        // For every particle in our state space
        int index = 0;
        for (Particle particle: Particles) {

            double cumalativeWeight = 0;
            double cumalativeWeightModifer = 0;

            // For every sensor model that we are considering
            for (SensorModel model: models) {

                // Get both the actual and simulated reading
                RealVector simulatedSensorValue = model.getSimulatedReading(particle.getState());
                RealVector actualSensorValue = model.getActualReading();

                double probability = readingDeltaProbability(actualSensorValue, simulatedSensorValue, model.getDOF());

                // Add the probability multiplied by the weight of the model.
                cumalativeWeight += probability * model.getWeightModifier();
                // Add the weight of this sensor model to the overall weight modifier
                cumalativeWeightModifer += model.getWeightModifier();

            }

            // Calculate the average weights of all the sensors and assign it to the particle
            particle.setWeight((cumalativeWeight/cumalativeWeightModifer)*particle.getWeight());

            // Add the particle with the updated weight back into our particle set.
            Particles.set(index, particle);
            index ++;
        }
    }

    /**
     * Resampling method that uses multinomial resampling
     */
    public void resampleParticles() {
        double sumWeights = 0;
        for (Particle particle: Particles) {
            sumWeights += particle.getWeight();
        }
        double probFactor = 1/sumWeights;

        // Normalize the weights off all particles
        int index = 0;
        for (Particle particle: Particles) {
            particle.setWeight(particle.getWeight()*probFactor);
            Particles.set(index, particle);
            index ++;
        }


    }

    /**
     * Gets the particle with the highest weight.
     * @return Particle of the highest weighted particle.
     */
    public Particle getBestParticle () {

        double highestWeight = 0;
        Particle bestParticle = new Particle();

        // Loop through all weights and get highest weight
        for (Particle particle : Particles) {
            double particleWeight = particle.getWeight();
            if (particleWeight > highestWeight) {
                bestParticle = particle;
                highestWeight = particleWeight;
            }
        }
        return bestParticle;
    }

    private double readingDeltaProbability(RealVector v1, RealVector v2, int DOF) {

        RealVector readingDelta = v1.subtract(v2);

        double probSensorGivenState = 0;
        if (DOF == 2){
            probSensorGivenState = distribution2DOF.density(readingDelta.getNorm());
        } else if ( DOF == 3 ) {
            probSensorGivenState = distribution3DOF.density(readingDelta.getNorm());
        }

        return probSensorGivenState;
    }
}

package org.firstinspires.ftc.teamcode.modules.gaeldrive.filters;

import android.provider.Telephony;

import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.geometry.Particle;

import java.util.HashMap;
import java.util.Map;

public class ParticleFilter {

    /**
     * Hashmap that stores all the particles in Integer/Particle pairs.
     */
    public HashMap<Integer, Particle> Particles = new HashMap<>();

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
     * @return
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

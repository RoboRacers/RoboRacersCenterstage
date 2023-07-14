package org.firstinspires.ftc.teamcode.modules.gaeldrive.geometry;

import android.os.Build;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

public class ParticleMap {

    public HashMap<Integer, Particle2d> Particles = new HashMap<>();
    public HashMap<Integer, Double> ParticleWeights = new HashMap<>();

    public ParticleMap () {

    }

    /**
     * Add a particle to the internal Hashmap.
     * @param particle
     */
    public void add(Particle2d particle) {
        Particles.put(particle.getId(), particle);
    }

    /**
     * Gets the particle with the highest weight.
     * @return Particle2d of the highest weighted particle.
     */
    public Particle2d getBestParticle () {

        /* Previous Code
        if (Build.VERSION.SDK_INT >= 0) {
            bestParticle = Particles.get(Collections.max(ParticleWeights.entrySet(), Map.Entry.comparingByValue()).getKey());
        }
        */

        double highestWeight = 0;
        Integer bestParticleKey = 0;

        // Loop through all weights and get highest weight
        for (Map.Entry<Integer,Particle2d> particle2dEntry : Particles.entrySet()) {
            double particleWeight = particle2dEntry.getValue().getWeight();
            if (particleWeight > highestWeight) {
                bestParticleKey = particle2dEntry.getKey();
                highestWeight = particleWeight;
            }

        }

        return Particles.get(bestParticleKey);

    }

    /**
     * Translate every particle
     * @param translationVector The translation vector that the other particles will be translated by.
     */
    public void translateParticles (RealVector translationVector) {

        for (Map.Entry<Integer,Particle2d> particle2dEntry : Particles.entrySet()) {
            Particle2d translatedParticle = particle2dEntry.getValue();
            translatedParticle.setState(translatedParticle.getState().add(translationVector));
            particle2dEntry.setValue(translatedParticle);

        }
    }

    public HashMap<Integer, Particle2d> getParticles() {
        return  this.Particles;
    }



}

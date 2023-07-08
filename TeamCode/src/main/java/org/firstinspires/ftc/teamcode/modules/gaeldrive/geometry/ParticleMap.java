package org.firstinspires.ftc.teamcode.modules.gaeldrive.geometry;

import java.util.HashMap;
import java.util.Map;

public class ParticleMap {

    HashMap<Integer, Particle2d> Particles = new HashMap<>();;
    HashMap<Integer, Double> ParticleWeights = new HashMap<>();;

    //TODO: Get Highest Weighed Particle
    public void add(Particle2d particle) {
        Particles.put(particle.getId(), particle);
        ParticleWeights.put(particle.getId(), particle.getWeight());

    }

    public void setParticleWeight(Integer id, Double weight) {
        ParticleWeights.put(id, weight);
    }

    public void getBestParticle () {
        Map.Entry<Integer, String> entryWithLargestKey = ParticleWeights
                .entrySet()
                .stream()
                .sorted(Map.Entry.comparingByKey(Comparator.reverseOrder()))
                .findFirst()
                .get();
    }



}

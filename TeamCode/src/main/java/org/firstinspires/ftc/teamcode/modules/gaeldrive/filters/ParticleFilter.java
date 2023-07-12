package org.firstinspires.ftc.teamcode.modules.gaeldrive.filters;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.geometry.Particle2d;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.geometry.ParticleMap;

import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ThreadLocalRandom;

public class ParticleFilter {
    List<Particle2d> Particles;

    List ParticleIds;

    ParticleMap particleMap;

    public void initializeParticles(int numParticles, Pose2d startingLocation) {
        Particles = null;
        ParticleIds = null;

        // Deviation Threshold for spawning new particles
        double min = -0.1;
        double max = 0.1;

        for(int i=0; i < numParticles; i++ ) {
            double deviation1 = ThreadLocalRandom.current().nextDouble(min, max);
            double deviation2 = ThreadLocalRandom.current().nextDouble(min, max);
            double deviation3 = ThreadLocalRandom.current().nextDouble(-0.05, 0.05);

            // Random Weight (For Testing Purposes) TODO: Remove Random Particle Weighting
            double weight = ThreadLocalRandom.current().nextDouble(0, 1);

            // Create the new pose
            Pose2d addedPose = new Pose2d(  startingLocation.getX() + deviation1,
                                            startingLocation.getY() + deviation2,
                                            startingLocation.getHeading() + deviation3);

            particleMap.add(new Particle2d(addedPose, weight, i, particleMap));


        }

    }

    public void translateParticles(RealVector translationVector) {
        for (Particle2d particle : Particles) {
            particle.setState(particle.getState().add(translationVector));
        }
    }

    public Pose2d getBestPose () {
        Pose2d bestPose = particleMap.getBestParticle().getPose();
        return bestPose;
    }

}

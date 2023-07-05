package org.firstinspires.ftc.teamcode.modules.gaeldrive.filters;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.geometry.Particle2d;

import java.util.List;
import java.util.concurrent.ThreadLocalRandom;

public class ParticleFilter {
    List<Particle2d> Particles;
    List ParticleIds;

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

            // Create the new pose
            Pose2d addedPose = new Pose2d(  startingLocation.getX() + deviation1,
                                            startingLocation.getY() + deviation2,
                                            startingLocation.getHeading() + deviation3);
            if (!(ParticleIds.contains(i))) {
                Particles.add(new Particle2d(addedPose, 0, i ));
            }

        }

    }

    public void translateParticles(RealVector translationVector) {
        for (Particle2d particle : Particles) {
            particle.setState(particle.getState().add(translationVector));
        }
    }

}

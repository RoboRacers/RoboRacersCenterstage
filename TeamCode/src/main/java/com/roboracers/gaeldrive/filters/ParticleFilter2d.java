package com.roboracers.gaeldrive.filters;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.roboracers.gaeldrive.LocalizationConstants;
import com.roboracers.gaeldrive.particles.Particle;
import com.roboracers.gaeldrive.particles.Particle2d;
import com.roboracers.gaeldrive.utils.PoseUtils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ThreadLocalRandom;

/**
 * Specialized Particle Filter class for 2d robot localization.
 * Note that this is technically a 3d (x,y,Heading) particle filter.
 */
public class ParticleFilter2d extends ParticleFilter {

    public ParticleFilter2d() {
    }

    /**
     * Initialize the particle set
     * @param numParticles The number of particles in the particle set
     * @param startingLocation The origin of the particles
     */
    public void initializeParticles(int numParticles, Pose2d startingLocation) {

        // Deviation Threshold for spawning new particles
        double min = -LocalizationConstants.POSITIONAL_COVARIANCES;
        double max = LocalizationConstants.POSITIONAL_COVARIANCES;
        double heading_min = -LocalizationConstants.ROTATIONAL_COVARIANCES;
        double heading_max = LocalizationConstants.ROTATIONAL_COVARIANCES;

        for(int i=0; i < numParticles; i++ ) {
            // Generate random deviances TODO: Make a more mathematical resampling system
            double deviation1 = ThreadLocalRandom.current().nextDouble(min, max);
            double deviation2 = ThreadLocalRandom.current().nextDouble(min, max);
            double deviation3 = ThreadLocalRandom.current().nextDouble(heading_min, heading_max);


            // Create the new pose
            Pose2d addedPose = new Pose2d(  startingLocation.getX() + deviation1,
                                            startingLocation.getY() + deviation2,
                                            startingLocation.getHeading() + deviation3);

            // Add the given particle back into the particle set
            add(new Particle2d(addedPose, 0, i));
        }

    }


    public Pose2d getBestPose () {
        return PoseUtils.vectorToPose(getBestParticle().getState());
    }

    public List<Pose2d> getParticlePoses (){
        List<Pose2d> poses = new ArrayList<>();
        HashMap<Integer, Particle> particles = getParticles();

        for (Map.Entry<Integer,Particle> particle2dEntry : particles.entrySet()) {
            poses.add(PoseUtils.vectorToPose(particle2dEntry.getValue().getState()));
        }

        return poses;
    }


}

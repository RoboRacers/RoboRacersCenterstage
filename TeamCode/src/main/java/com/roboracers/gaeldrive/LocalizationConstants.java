package com.roboracers.gaeldrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Class full of constants to tune the localization.
 */
public class LocalizationConstants {

    //TODO: Tune Localization Constants
    /**
     * Number of particles in the particle filter. More particles means higher accuracy,
     * but worse performance. Tune to a value you feel right with.
     */
    public static int PARTICLE_COUNT = 20;

    /**
     * The default start pose of the robot. Feel free to change this or just call
     * "drive.setPoseEstimate()" in the start of your autonomous cycle.
     */
    public static Pose2d START_POSE = new Pose2d(0,0,0);

    /**
     * The positional deviation in which the particles are spawned in relation to their X and Y coordinates.
     */
    public static double POSITIONAL_COVARIANCES = 72;

    /**
     * The rotational deviation in which the particles are spawned in relation to heading.
     */
    public static double ROTATIONAL_COVARIANCES = Math.toRadians(180);

    /**
     * After how many update cycles the particle set is resampled. Lower values mean more performance,
     * but at the cost of accuracy.
     */
    public static int RESAMPLING_CYCLES = 2;

    /**
     * How much the robot has to move for the particles to be updated.
     */
    public static double UPDATE_THRESHOLD = 0;

    public static boolean TESTING = false;
}

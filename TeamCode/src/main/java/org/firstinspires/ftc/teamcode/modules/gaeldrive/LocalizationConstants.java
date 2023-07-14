package org.firstinspires.ftc.teamcode.modules.gaeldrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Class ful of constants to tune the localization.
 */
public class LocalizationConstants {

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
}

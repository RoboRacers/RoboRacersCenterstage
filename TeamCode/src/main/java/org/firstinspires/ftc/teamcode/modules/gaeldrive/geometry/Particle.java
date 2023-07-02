package org.firstinspires.ftc.teamcode.modules.gaeldrive.geometry;

import com.acmerobotics.roadrunner.geometry.Pose2d;


/**
 * A class to represent a pose and a weight, the building block of the particle filter.
 */
public class Particle {

    Pose2d pose;
    float weight;

    public Particle(Pose2d pose2d, float weight) {
        this.pose = pose2d;
        this.weight = weight;
    }

    Pose2d getPose(){
        return this.pose;
    }
    float getWeight(){
        return this.weight;
    }

}

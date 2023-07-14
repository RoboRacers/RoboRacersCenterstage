package org.firstinspires.ftc.teamcode.modules.gaeldrive.geometry;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.utils.PoseUtils;


/**
 * Extension of the Particle class, intended for used with a 2d robot localization system.
 * Note that the state of this particle is actually 3 dimensions (x,y, heading).
 * @see Particle
 */
public class Particle2d extends Particle {

    Pose2d pose;
    ParticleMap map;

    public Particle2d(Pose2d pose2d, double weight, Integer id, ParticleMap map) {
        this.pose = pose2d;
        this.weight = weight;
        this.id = id;
        this.map = map;

        PoseToState();
    }


    public void setPose(Pose2d newPose){
        pose = newPose;
        PoseToState();
    }
    public Pose2d getPose(){
        return this.pose;
    }

    @Override
    public void setState(RealVector newState) {
        state = newState;
        StateToPose();
    }

    @Override
    public void setWeight(Double weight) {
        this.weight = weight;
    }

    void PoseToState () {
        state = PoseUtils.poseToVecor(pose);
    }

    void StateToPose () {
        pose = PoseUtils.vectorToPose(state);
    }


}

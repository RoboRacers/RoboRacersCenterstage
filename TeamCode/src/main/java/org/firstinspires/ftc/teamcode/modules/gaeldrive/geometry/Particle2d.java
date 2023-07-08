package org.firstinspires.ftc.teamcode.modules.gaeldrive.geometry;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.utils.PoseUtils;


/**
 * A class to represent a pose and a weight, the building block of the particle filter.
 */
public class Particle2d {

    Pose2d pose;
    RealVector state;
    public double weight;
    public Integer id;
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

    public void setState(RealVector newState) {
        state = newState;
        StateToPose();
    }

    public RealVector getState() {
        return state;
    }

    public double getWeight(){
        return this.weight;
    }

    public void setWeight(Double weight) {
        this.weight = weight;
        map.setParticleWeight(this.getId(), this.weight);
    }

    public int getId() { return this.id; }

    void PoseToState () {
        state = PoseUtils.poseToVecor(pose);
    }

    void StateToPose () {
        pose = PoseUtils.vectorToPose(state);
    }


}

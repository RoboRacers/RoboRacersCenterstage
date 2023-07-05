package org.firstinspires.ftc.teamcode.modules.gaeldrive.geometry;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.utils.Pose2dUtils;


/**
 * A class to represent a pose and a weight, the building block of the particle filter.
 */
public class Particle2d {

    Pose2d pose;
    RealVector state;
    public float weight;
    public int id;

    public Particle2d(Pose2d pose2d, float weight, int id) {
        this.pose = pose2d;
        this.weight = weight;
        this.id = id;

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
    public int getId() { return this.id; }

    void PoseToState () {
        state = Pose2dUtils.poseToVecor(pose);
    }

    void StateToPose () {
        pose = Pose2dUtils.vectorToPose(state);
    }


}

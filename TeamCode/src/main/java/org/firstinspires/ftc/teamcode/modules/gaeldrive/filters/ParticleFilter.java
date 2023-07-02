package org.firstinspires.ftc.teamcode.modules.gaeldrive.filters;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class ParticleFilter {
    RealVector constants = new ArrayRealVector(new double[] { 1, -2, 1 }, false);
    //RealVector Particles = new ArrayRealVector(new Pose2d[] { 1, -2, 1 }, false);

}

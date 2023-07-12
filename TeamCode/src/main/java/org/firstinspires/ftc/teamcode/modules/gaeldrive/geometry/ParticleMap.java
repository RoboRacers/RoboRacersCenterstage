package org.firstinspires.ftc.teamcode.modules.gaeldrive.geometry;

import android.os.Build;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

public class ParticleMap {

    public HashMap<Integer, Particle2d> Particles = new HashMap<>();
    public HashMap<Integer, Double> ParticleWeights = new HashMap<>();

    public ParticleMap () {

    }

    //TODO: Get Highest Weighed Particle
    public void add(Particle2d particle) {
        Particles.put(particle.getId(), particle);
        ParticleWeights.put(particle.getId(), particle.getWeight());

    }

    public void setParticleWeight(Integer id, Double weight) {
        ParticleWeights.put(id, weight);
    }

    public Particle2d getBestParticle () {

        // Default value
        Particle2d bestParticle = new Particle2d(new Pose2d(0,0,0), 0, 0, this);

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            bestParticle = Particles.get(Collections.max(ParticleWeights.entrySet(), Map.Entry.comparingByValue()).getKey());
        }

        return bestParticle;

    }

    public void translateParticles (RealVector translationVector) {
        for (Map.Entry<Integer,Particle2d> particle2dEntry : Particles.entrySet()) {
            Particle2d translatedParticle = particle2dEntry.getValue();
            translatedParticle.setState(translatedParticle.getState().add(translationVector));
            particle2dEntry.setValue(translatedParticle);

            //Draw particle on FtcDash
        }
    }

    public HashMap<Integer, Particle2d> getParticles() {
        return  this.Particles;
    }



}

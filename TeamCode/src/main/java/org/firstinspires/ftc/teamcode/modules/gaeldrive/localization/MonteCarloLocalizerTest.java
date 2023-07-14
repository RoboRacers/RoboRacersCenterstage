package org.firstinspires.ftc.teamcode.modules.gaeldrive.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.teamcode.modules.gaeldrive.filters.ParticleFilter2d;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.motion.TestMotionModel;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors.TestSensorBuffer;
import org.jetbrains.annotations.TestOnly;


/**
 * Localizer that uses Monte Carlo Localization (MCL) to get the position of the robot,
 * given a set of sensor values
 */
@TestOnly
public class MonteCarloLocalizerTest implements Localizer {

    Pose2d  poseEstimate =  new Pose2d(0,0,0);
    int particleCount = 20;
    FtcDashboard dashboard;

    ParticleFilter2d particleFilter2d;
    TestMotionModel motionModel;

    public MonteCarloLocalizerTest(){
        TestSensorBuffer.init();
        particleFilter2d = new ParticleFilter2d();
        dashboard = FtcDashboard.getInstance();
        particleFilter2d.initializeParticles(particleCount, poseEstimate);
        motionModel = new TestMotionModel(poseEstimate);
    }

    /**
     * Getting and Setting the PoseEstimate
     */
    @Override
    public Pose2d getPoseEstimate() {
        return this.poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        this.poseEstimate = pose2d;
        particleFilter2d.initializeParticles(particleCount, poseEstimate);
    }

    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    /**
     * Runs one localization cycle.
     */
    @Override
    public void update() {
        TestSensorBuffer.update();
        particleFilter2d.translateParticles(motionModel.getTranslationVector());
        this.poseEstimate = particleFilter2d.getBestPose();

        /*
        List<Pose2d> poses = particleFilter.getParticlePoses();


        TelemetryPacket packet = new TelemetryPacket();

        for (Pose2d drawnPose : poses) {
            packet.fieldOverlay().strokeCircle(drawnPose.getX(), drawnPose.getY(), 0.5);
            dashboard.sendTelemetryPacket(packet);
        }
        */

    }

}

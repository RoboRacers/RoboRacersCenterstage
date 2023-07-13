package org.firstinspires.ftc.teamcode.modules.gaeldrive.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.linear.MatrixUtils;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.filters.ParticleFilter;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.motion.TrackingWheelMotionModel;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors.SensorBuffer;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors.TestSensorBuffer;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.utils.PoseUtils;

import java.util.List;


/**
 * Localizer that uses Monte Carlo Localization (MCL) to get the position of the robot,
 * given a set of sensor values
 */
public class MonteCarloLocalizerTest implements Localizer {

    Pose2d  poseEstimate = new Pose2d(0,0,0);
    int particleCount = 20;
    FtcDashboard dashboard;

    ParticleFilter particleFilter;
    TrackingWheelMotionModel motionModel;

    public MonteCarloLocalizerTest(){
        TestSensorBuffer.init();
        particleFilter = new ParticleFilter();
        dashboard = FtcDashboard.getInstance();
        particleFilter.initializeParticles(this.particleCount, this.poseEstimate);
        motionModel = new TrackingWheelMotionModel(poseEstimate);
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
        particleFilter.initializeParticles(particleCount, poseEstimate);
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
        particleFilter.translateParticles(PoseUtils.poseToVecor(TestSensorBuffer.mockPoseEstimate));
        poseEstimate = particleFilter.getBestPose();

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

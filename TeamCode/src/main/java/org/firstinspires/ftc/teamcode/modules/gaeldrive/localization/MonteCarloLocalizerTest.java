package org.firstinspires.ftc.teamcode.modules.gaeldrive.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.filters.ParticleFilter;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.motion.TrackingWheelMotionModel;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors.SensorBuffer;

import java.util.List;


/**
 * Localizer that uses Monte Carlo Localization (MCL) to get the position of the robot,
 * given a set of sensor values
 */
public class MonteCarloLocalizerTest implements Localizer {

    Pose2d  poseEstimate = new Pose2d(0,0,0);
    int particleCount = 20;
    FtcDashboard dashboard;
    Telemetry telemetry;

    ParticleFilter particleFilter;
    TrackingWheelMotionModel motionModel;

    public MonteCarloLocalizerTest(HardwareMap hardwareMap){
        SensorBuffer.init(hardwareMap);
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
        if (pose2d != null) {
            this.poseEstimate = pose2d;
        } else {
            this.poseEstimate = new Pose2d(0,0,0);;
        }
        particleFilter.initializeParticles(particleCount, poseEstimate);
    }

    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    /**
     * Runs one localization cycle
     * This function is inherited from the Localizer class and is used by the drive object.
     */
    @Override
    public void update() {
        SensorBuffer.update();
        particleFilter.translateParticles(motionModel.getTranslationVector());
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

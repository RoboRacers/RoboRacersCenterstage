package org.firstinspires.ftc.teamcode.modules.gaeldrive.readings;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.modules.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.LocalizationConstants;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.motion.MotionModel;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.motion.TrackingWheelMotionModel;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors.DistanceSensorModel;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.sensors.SensorModel;

import java.util.ArrayList;
import java.util.List;

/**
 * Global buffer to store and load sensor values.
 */
public class StandardSensorStack implements SensorStack {

    public static TrackingWheelMotionModel trackingWheelMotionModel;
    public static DistanceSensorModel distanceSensorModel;

    static public void init(HardwareMap hardwareMap) {
        trackingWheelMotionModel = new TrackingWheelMotionModel(LocalizationConstants.START_POSE, hardwareMap);
        distanceSensorModel = new DistanceSensorModel(hardwareMap.get(DistanceSensor.class, "range"));
    }

    public static void update(){
        trackingWheelMotionModel.update();
    }

    public static List<SensorModel> getSensorModels() {
        List<SensorModel> sensorModels = new ArrayList<>();
        sensorModels.add(distanceSensorModel);
       return sensorModels;
    }

}

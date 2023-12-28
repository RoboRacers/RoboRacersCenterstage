package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

public class Transform3d {

    VectorF translation;
    Quaternion rotation;

    private final static VectorF zeroVector = new VectorF(0,0,0);

    public Transform3d() {
        translation = new VectorF(0f,0f,0f);
        rotation = new Quaternion();
    }

    public Transform3d(VectorF t, Quaternion Q) {
        translation = t;
        rotation = Q;
    }

    public Transform3d unaryMinusInverse() {
        Quaternion Q = rotation.inverse();
        VectorF t = rotation.applyToVector(zeroVector.subtracted(translation));
        return new Transform3d(t,Q);
    }

    public Transform3d plus(Transform3d P) {
        VectorF t = translation.added(rotation.applyToVector(P.translation));
        Quaternion Q = rotation.multiply(P.rotation, System.nanoTime());
        return new Transform3d(t,Q);
    }

    public double getZ() {
        return Math.atan2(2.0*(rotation.y*rotation.z + rotation.w*rotation.x),
                rotation.w*rotation.w - rotation.x*rotation.x - rotation.y*rotation.y + rotation.z*rotation.z);
    }

    public Pose2d toPose2d() {
        return new Pose2d(
                translation.get(0),
                translation.get(1),
                this.getZ()
        );
    }
}

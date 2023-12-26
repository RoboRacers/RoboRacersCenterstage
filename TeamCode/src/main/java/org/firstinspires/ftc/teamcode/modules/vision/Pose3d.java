package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.exception.DimensionMismatchException;
import org.apache.commons.math3.exception.MathIllegalArgumentException;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class Pose3d {

    VectorF translation;
    MatrixF rotation;

    private final static VectorF zeroVector = new VectorF(0,0,0);
    private final static MatrixF zeroMatrix = new GeneralMatrixF(3,3);

    public Pose3d () {
        translation = new VectorF(0f,0f,0f);
        rotation = new GeneralMatrixF(3,3);
    }

    public Pose3d (VectorF t, MatrixF R) {
        assert t.length() == 3 && R.numCols() == 3 && R.numRows() == 3;
        translation = t;
        rotation = R;
    }

    public Pose3d unaryMinusInverse() {
        MatrixF R = zeroMatrix.subtracted(rotation);
        VectorF t = rotation.multiplied(zeroVector.subtracted(translation));
        return new Pose3d(t,R);
    }

    public Pose3d plus(Pose3d P) {
        VectorF t = translation.added(rotation.multiplied(P.translation));
        MatrixF R = rotation.multiplied(P.rotation);
        return new Pose3d(t,R);
    }

    public Pose2d toPose2d() {
        return new Pose2d(
                translation.get(0),
                translation.get(1),
                Math.atan2(-rotation.get(0,2), rotation.get(2,2))
        );
    }
}

package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.autonomous.AutoopStateMachines;

public class RoadrunnerPointDataset {

    private SampleMecanumDrive lDrive;
    private MultipleTelemetry ltelementry;

    public final Pose2d S0_POS = new Pose2d(36, -64.5, Math.toRadians(-270));
    public final Pose2d S1_POS = new Pose2d(36, 64.5, Math.toRadians(270));
    public final Pose2d S2_POS = new Pose2d(-36, 64.5, Math.toRadians(270));
    public final Pose2d S3_POS = new Pose2d(-36, -64.5, Math.toRadians(-270));

    int stack1 = -300;
    final int liftLow = 0;
    final int liftHigherThanLow = -750;
    final int liftMid = -1075;
    final int liftHigh = -1350;

    DcMotorEx lmotorLeft;
    DcMotorEx lmotorRight;
    Servo lclaw;

    final double close = 0.25;
    final double open = 0;


    public RoadrunnerPointDataset(SampleMecanumDrive drive, MultipleTelemetry telemetry, DcMotorEx motorRight, DcMotorEx motorLeft, Servo claw) {
        lDrive = drive;
        ltelementry = telemetry;

        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lmotorLeft = motorLeft;
        lmotorRight = motorRight;

        lclaw = claw;

    }



    AutoopStateMachines AutoCaller;

    public void test() {
        Pose2d StartPose = new Pose2d(36, -64.5, Math.toRadians(-270));
        lDrive.setPoseEstimate(StartPose);

        Trajectory traj1 = lDrive.trajectoryBuilder(StartPose)
                .lineTo(new Vector2d(36, -35))
                .build();
        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(12, -35))
                .build();

        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
    }

    public void S0PP1 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S0_POS)
                .lineTo(new Vector2d(36, -35))
                .build();

        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(12, -35))
                .build();
        Trajectory traj3 = lDrive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(12, -24))
                .build();

        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj3);
    }

    public void S0PP2 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S0_POS)
                .lineTo(new Vector2d(36, -24))
                .build();
        lDrive.followTrajectory(traj1);
    }

    public void S0PP3 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S0_POS)
                .lineTo(new Vector2d(36, -35))
                .build();

        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(57.75, -35))
                .build();
        Trajectory traj3 = lDrive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(57.75, -24))
                .build();

        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj3);
    }

    public void S1PP1 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S1_POS)
                .lineTo(new Vector2d(36, 35))
                .build();

        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(58, 35))
                .build();
        Trajectory traj3 = lDrive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(58, 24))
                .build();

        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj3);
    }

    public void S1PP2 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S1_POS)
                .lineTo(new Vector2d(36, 24))
                .build();
        lDrive.followTrajectory(traj1);
    }

    public void S1PP3 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S1_POS)
                .lineTo(new Vector2d(36, 36))
                .build();

        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(12, 36))
                .build();
        Trajectory traj3 = lDrive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(12, 24))
                .build();

        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj3);
        }

    public void S2PP1 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S2_POS)
                .lineTo(new Vector2d(-36, 35))
                .build();
        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(-12, 35))
                .build();
        Trajectory traj3 = lDrive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(-12, 24))
                .build();

        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj3);
    }

    public void S2PP2 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S2_POS)
                .lineTo(new Vector2d(-36, 24))
                .build();
        lDrive.followTrajectory(traj1);
    }

    public void S2PP3 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S2_POS)
                .lineTo(new Vector2d(-36, 35))
                .build();

        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(-57.75, 35))
                .build();
        Trajectory traj3 = lDrive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(-57.75, 24))
                .build();
        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj3);
    }

    public void S3PP1 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S3_POS)
                .lineTo(new Vector2d(-36, -36))
                .build();
        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(-57.75, -36))
                .build();
        Trajectory traj3 = lDrive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(-57.75, -24))
                .build();
        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj3);
    }

    public void S3PP2 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S3_POS)
                .lineTo(new Vector2d(-36, -24))
                .build();
        lDrive.followTrajectory(traj1);
    }

    public void S3PP3 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S3_POS)
                .lineTo(new Vector2d(-36, -36))
                .build();
        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(-12, -36))
                .build();
        Trajectory traj3 = lDrive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(-12, -24))
                .build();
        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj3);
    }

    // Medium Cycles Trajectories Init
    TrajectorySequence trajSeq0;
    Trajectory traj0;
    TrajectorySequence trajSeq1;
    Trajectory traj1;

    TrajectorySequence trajSeq2;
    Trajectory traj2;
    Trajectory traj3;
    TrajectorySequence trajSeq3;
    Trajectory traj4;
    TrajectorySequence trajSeq4;
    TrajectorySequence traj5;
    Trajectory traj6;
    TrajectorySequence trajSeq9;
    TrajectorySequence trajSeq6;
    TrajectorySequence traj7;
    Trajectory traj8;
    TrajectorySequence traj10;

    double preloadXmodifier = -1.5;
    int preloadYmodifier = 1;

    double stackXmodifier = -0.25;
    int stackYmodifier = 0;

    double cycleXmodifier = -1.5;
    int cycleYmodifier = 1;

    int cycleNumber = 0;



    public void HighPreloadRightV2 () {
        Pose2d StartPose = new Pose2d(-33, 64.5, Math.toRadians(270));
        lDrive.setPoseEstimate(StartPose);

        // Starting Shift
        trajSeq0 = lDrive.trajectorySequenceBuilder(StartPose)
                .strafeTo(new Vector2d(-36, 64.5))
                .build();

        // Preload
        traj1 = lDrive.trajectoryBuilder(trajSeq0.end())
                .lineTo(new Vector2d(-36, 15),
                        lDrive.getVelocityConstraint(85, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        lDrive.getAccelerationConstraint(50)
                )
                .addSpatialMarker(new Vector2d(-36, 45),() -> {
                    claw(close);
                })
                .addSpatialMarker(new Vector2d(-36, 20),() -> {
                    ArmPosition(-170, 1);
                })
                .build();

        // Shift to Pole
        traj2 = lDrive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(-25, 15))
                .addSpatialMarker(new Vector2d(-24, 15),() -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();

        // Drop-Off
        trajSeq3 = lDrive.trajectorySequenceBuilder(traj2.end())

                .lineTo(new Vector2d(-25+preloadXmodifier, 11+preloadYmodifier),
                        lDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        lDrive.getAccelerationConstraint(25)
                        )
                .waitSeconds(1)
                .addTemporalMarker(1, () -> {
                    ArmPosition(liftHigh+350, 1);
                })
                .addTemporalMarker(1.5, () -> {
                    claw(open);
                })
                .addTemporalMarker(1.75, () -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();


        // Backing up
        traj4 = lDrive.trajectoryBuilder(trajSeq3.end())
                .lineTo(new Vector2d(-24+preloadYmodifier, 19))
                .addSpatialMarker(new Vector2d(-24+preloadXmodifier, 17),() -> {
                    ArmPosition(stack1, 1);
                    claw(open);
                })
                .build();

        // Go to stack
        traj5 = lDrive.trajectorySequenceBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(-60+stackXmodifier, 23.5+stackYmodifier, Math.toRadians(180))
                )
                .waitSeconds(0.5)
                .addTemporalMarker(0, () -> {
                    claw(open);
                })
                .addTemporalMarker(2, () -> {
                    claw(close);
                })
                .addTemporalMarker(2.5, () -> {
                    ArmPosition(-500, 0.8);
                })
                .build();

        traj10 = lDrive.trajectorySequenceBuilder(traj5.end())
                .strafeTo(new Vector2d(-60+stackXmodifier, 23.5+stackYmodifier+2))
                .build();

        // Go to pole
        trajSeq6 = lDrive.trajectorySequenceBuilder(traj10.end())
                .lineToLinearHeading(new Pose2d(-24, 20, Math.toRadians(270)))
                .addSpatialMarker(new Vector2d(-50, 20),() -> {
                    ArmPosition(liftHigh, 0.75);
                })
                .build();


        // Drop off Cone
        traj7 = lDrive.trajectorySequenceBuilder(trajSeq6.end())
                .lineTo(new Vector2d(-24+cycleXmodifier, 11+cycleYmodifier),
                        lDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        lDrive.getAccelerationConstraint(25)
                )
                .waitSeconds(1)
                .addTemporalMarker(1, () -> {
                    ArmPosition(liftHigh+350, 1);
                })
                .addTemporalMarker(1.5, () -> {
                    claw(open);
                })
                .addTemporalMarker(1.75, () -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();

        traj8 = lDrive.trajectoryBuilder(traj7.end())
                .lineTo(new Vector2d(-24+cycleXmodifier, 19))
                .addSpatialMarker(new Vector2d(-24+cycleXmodifier, 17),() -> {
                    if (cycleNumber == 0){
                        ArmPosition(stack1, 1);
                    } else if (cycleNumber == 1){
                        ArmPosition(liftLow, 1);
                    }

                })
                .build();

        lDrive.followTrajectorySequence(trajSeq0);
        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectorySequence(trajSeq3);
        lDrive.followTrajectory(traj4);

        lDrive.followTrajectorySequence(traj5);
        lDrive.followTrajectorySequence(traj10);
        lDrive.followTrajectorySequence(trajSeq6);
        lDrive.followTrajectorySequence(traj7);
        lDrive.followTrajectory(traj8);

        stack1 = stack1 + 30;
        cycleNumber = cycleNumber + 1;

        lDrive.followTrajectorySequence(traj5);
        lDrive.followTrajectorySequence(traj10);
        lDrive.followTrajectorySequence(trajSeq6);
        lDrive.followTrajectorySequence(traj7);
        lDrive.followTrajectory(traj8);

        lDrive.update();
    }

    public void HighCycleRightV3 () {
        Pose2d StartPose = new Pose2d(-33, 64.5, Math.toRadians(270));
        lDrive.setPoseEstimate(StartPose);

        // Starting Shift
        trajSeq0 = lDrive.trajectorySequenceBuilder(StartPose)
                .splineTo(new Vector2d(-36, 40), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    ArmPosition(liftHigh, 0.60);
                })
                .splineTo(new Vector2d(-29, 8), Math.toRadians(270+38))
                .build();

        // Go to stack
        trajSeq1 = lDrive.trajectorySequenceBuilder(trajSeq0.end())
                .waitSeconds(2)
                .addTemporalMarker(1, () -> {
                    claw(open);
                })
                .back(1)
                .build();
        traj1 = lDrive.trajectoryBuilder(trajSeq1.end(), true)
                .splineTo(new Vector2d(-36, 24), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    ArmPosition(stack1, 0.75);
                    claw(open);
                })
                .build();
        trajSeq2 = lDrive.trajectorySequenceBuilder(traj1.end())
                .splineTo(new Vector2d(-60+stackXmodifier, 23), Math.toRadians(180))
                .waitSeconds(2)
                .addTemporalMarker(4, () -> {
                    claw(close);
                })
                .addTemporalMarker(5, () -> {
                    ArmPosition(-500, 0.75);
                })
                .build();
        traj3 = lDrive.trajectoryBuilder(trajSeq2.end(), true)
                .splineTo(new Vector2d(-38, 24), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    ArmPosition(liftHigh, 0.75);
                })
                .build();
        trajSeq4 = lDrive.trajectorySequenceBuilder(traj3.end())
                .splineTo(new Vector2d(-29, 8), Math.toRadians(270+38))
                .waitSeconds(3)
                .addTemporalMarker(4, () -> {
                    claw(open);
                })
                .build();


        lDrive.followTrajectorySequence(trajSeq0);
        lDrive.followTrajectorySequence(trajSeq1);
        lDrive.followTrajectory(traj1);
        lDrive.followTrajectorySequence(trajSeq2);
        lDrive.followTrajectory(traj3);
        lDrive.followTrajectorySequence(trajSeq4);


        lDrive.update();
    }

    public void HighCycleRightV4 () {
        Pose2d StartPose = new Pose2d(-33, 64.5, Math.toRadians(270));
        lDrive.setPoseEstimate(StartPose);

        // Starting Shift
        trajSeq0 = lDrive.trajectorySequenceBuilder(StartPose)
                .strafeTo(new Vector2d(-36, 64.5))
                .build();

        // Preload
        traj1 = lDrive.trajectoryBuilder(trajSeq0.end())
                .lineTo(new Vector2d(-36, 15),
                        lDrive.getVelocityConstraint(85, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        lDrive.getAccelerationConstraint(50)
                )
                .addSpatialMarker(new Vector2d(-36, 45),() -> {
                    claw(close);
                })
                .addSpatialMarker(new Vector2d(-36, 20),() -> {
                    ArmPosition(-170, 1);
                })
                .build();

        // Shift to Pole
        traj2 = lDrive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(-25, 15))
                .addSpatialMarker(new Vector2d(-24, 15),() -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();

        // Drop-Off
        trajSeq3 = lDrive.trajectorySequenceBuilder(traj2.end())

                .lineTo(new Vector2d(-25+preloadXmodifier, 11+preloadYmodifier),
                        lDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        lDrive.getAccelerationConstraint(25)
                )
                .waitSeconds(1)
                .addTemporalMarker(1, () -> {
                    ArmPosition(liftHigh+350, 1);
                })
                .addTemporalMarker(1.5, () -> {
                    claw(open);
                })
                .addTemporalMarker(1.75, () -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();


        // Backing up
        traj4 = lDrive.trajectoryBuilder(trajSeq3.end())
                .lineTo(new Vector2d(-24+preloadYmodifier, 19))
                .addSpatialMarker(new Vector2d(-24+preloadXmodifier, 17),() -> {
                    ArmPosition(stack1, 1);
                    claw(open);
                })
                .build();


        lDrive.followTrajectorySequence(trajSeq0);
        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectorySequence(trajSeq3);
        lDrive.followTrajectory(traj4);
        lDrive.update();
    }

    public void HighPreloadLeftV2 () {
        Pose2d StartPose = new Pose2d(38, 64.5, Math.toRadians(270));
        lDrive.setPoseEstimate(StartPose);

        // Shift
        trajSeq0 = lDrive.trajectorySequenceBuilder(StartPose)
                .strafeTo(new Vector2d(34.5, 64.5))
                .build();

        // Preload
        traj1 = lDrive.trajectoryBuilder(trajSeq0.end())
                .lineTo(new Vector2d(34.5, 12))
                .addSpatialMarker(new Vector2d(34.5, 45),() -> {
                    claw(close);
                })
                .addSpatialMarker(new Vector2d(34.5, 20),() -> {
                    ArmPosition(stack1+100, 1);
                })
                .build();
        traj2 = lDrive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(19.5, 12))
                .addSpatialMarker(new Vector2d(24, 12),() -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();
        trajSeq3 = lDrive.trajectorySequenceBuilder(traj2.end())
                .lineTo(new Vector2d(19.5, 10.5))
                .waitSeconds(2)
                .addTemporalMarker(3, () -> {
                    ArmPosition(liftHigh+350, 1);
                })
                .addTemporalMarker(4, () -> {
                    claw(open);
                })
                .waitSeconds(2)
                .addTemporalMarker(5, () -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();
        traj4 = lDrive.trajectoryBuilder(trajSeq3.end())
                .lineTo(new Vector2d(19.5, 16))
                .addSpatialMarker(new Vector2d(24, 16),() -> {
                    ArmPosition(liftLow, 1);
                })
                .build();

        lDrive.followTrajectorySequence(trajSeq0);
        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectorySequence(trajSeq3);
        lDrive.followTrajectory(traj4);

        lDrive.update();
    }

    /* Parking after Preload */

    public void PreloadParkingRightPP1 () {
        lDrive.setPoseEstimate(new Pose2d(-24, 16, Math.toRadians(270)));
        Trajectory parkingtraj1 = lDrive.trajectoryBuilder(new Pose2d(-24, 16, Math.toRadians(270)))
                .strafeTo(new Vector2d(-13, 16))
                .build();
        lDrive.followTrajectory(parkingtraj1);
        lDrive.update();
    }

    public void PreloadParkingRightPP2 () {
        lDrive.setPoseEstimate(new Pose2d(-24, 16, Math.toRadians(270)));
        Trajectory parkingtraj2 = lDrive.trajectoryBuilder(new Pose2d(-24, 16, Math.toRadians(270)))
                .strafeTo(new Vector2d(-36, 16))
                .build();
        lDrive.followTrajectory(parkingtraj2);
        lDrive.update();
    }

    public void PreloadParkingRightPP3 () {
        lDrive.setPoseEstimate(new Pose2d(-24, 16, Math.toRadians(90)));
        Trajectory parkingtraj3 = lDrive.trajectoryBuilder(new Pose2d(-24, 16, Math.toRadians(270)))
                .strafeTo(new Vector2d(-60, 16))
                .build();
        lDrive.followTrajectory(parkingtraj3);
        lDrive.update();
    }

    public void PreloadParkingLeftPP1 () {
        lDrive.setPoseEstimate(new Pose2d(24, 14, Math.toRadians(270)));
        Trajectory parkingtraj1 = lDrive.trajectoryBuilder(new Pose2d(24, 14, Math.toRadians(270)))
                .strafeTo(new Vector2d(62, 14))
                .build();
        lDrive.followTrajectory(parkingtraj1);
        lDrive.update();
    }

    public void PreloadParkingLeftPP2 () {
        lDrive.setPoseEstimate(new Pose2d(24, 14, Math.toRadians(270)));
        Trajectory parkingtraj2 = lDrive.trajectoryBuilder(new Pose2d(24, 14, Math.toRadians(270)))
                .strafeTo(new Vector2d(36, 14))
                .build();
        lDrive.followTrajectory(parkingtraj2);
        lDrive.update();
    }

    public void PreloadParkingLeftPP3 () {
        lDrive.setPoseEstimate(new Pose2d(24, 14, Math.toRadians(270)));
        Trajectory parkingtraj3 = lDrive.trajectoryBuilder(new Pose2d(24, 14, Math.toRadians(270)))
                .strafeTo(new Vector2d(12, 14))
                .build();
        lDrive.followTrajectory(parkingtraj3);
        lDrive.update();
    }

    public void DemoAuto1 () {
        lDrive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
        Trajectory traj1 = lDrive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .forward(60)
                .build();
        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .addDisplacementMarker(() -> {
                    ArmPosition(liftMid, 0.75);
                })
                .strafeRight(24)
                .build();
        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.update();
    }

    public void DemoAuto2 () {
        lDrive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
        Trajectory traj1 = lDrive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .forward(48)
                .build();
        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .addDisplacementMarker(()-> {
                    ArmPosition(liftMid, 0.7);
                })
                .strafeRight(24)
                .build();
        Trajectory traj3 = lDrive.trajectoryBuilder(traj2.end())
                .forward(12)
                .build();
        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj3);
        lDrive.turn(Math.toRadians(360+45));
        lDrive.update();
    }

    public void DemoAuto3 () {
        lDrive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
        Trajectory traj1 = lDrive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .forward(50)
                .build();
        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .addDisplacementMarker(()-> {
                    ArmPosition(liftMid, 0.7);
                })
                .strafeLeft(22)
                .build();
        Trajectory traj3 = lDrive.trajectoryBuilder(traj2.end())
                .forward(12)
                .build();
        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj3);
        lDrive.turn(Math.toRadians(-360-45));
        lDrive.update();
    }

    // Functions

    public void ArmPosition(int pos, double speed) {
        lmotorRight.setTargetPosition(-pos);
        lmotorLeft.setTargetPosition(-pos);
        lmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lmotorLeft.setPower(speed);
        lmotorRight.setPower(speed);
    }

    public void claw(double pos) {
        lclaw.setPosition(pos);
    }


}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.subsystems.Slides;
import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.subsystems.Intake;
import org.firstinspires.ftc.teamcode.modules.subsystems.Launcher;

/**
 * This class contains the declaration and setup for all motors and cameras on the robot.
 * To use this class in an opmode, declare:
 * RobotCore robot = new RobotCore(hardwareMap);
 * You can call motors and such through:
 * robot.componentName.command();
 */

public class RobotCore {

    /*
     * Declare the different subsystems of the robot here.
     */
    public SampleMecanumDrive drive;

    public Slides slides;

    public Intake intake;

    public Launcher launcher;

    public HardwareMap hardwareMap;

    /**
     * Constructor for the RobotCore class. Runs the setup for all subsystems.
     * @param hardwareMap
     */
    public RobotCore(HardwareMap hardwareMap){

        this.hardwareMap = hardwareMap;

        /* Initialize Subsystems */
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slides = new Slides(hardwareMap);

        intake = new Intake(hardwareMap);

        launcher = new Launcher(hardwareMap);

    }


}

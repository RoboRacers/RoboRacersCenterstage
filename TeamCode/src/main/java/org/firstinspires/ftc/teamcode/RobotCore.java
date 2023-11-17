package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.subsystems.Intake;
import org.firstinspires.ftc.teamcode.modules.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.modules.subsystems.Slides;
import org.firstinspires.ftc.teamcode.modules.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This class contains the declaration and setup for all motors and cameras on the robot.
 * To use this class in an opmode, declare:
 * RobotCore robot = new RobotCore(hardwareMap);
 * You can call motors and such through:
 * robot.componentName.command();
 */

public class RobotCore extends Subsystem {

    /*
     * Declare the different subsystems of the robot here.
     */
    public SampleMecanumDrive drive;

    public Intake intake;

    public Launcher launcher;

    public Slides slides;


    public List<Subsystem> subsystems;

    /**
     * Constructor for the RobotCore class. Runs the setup for all subsystems.
     * @param hardwareMap
     */
    public RobotCore(HardwareMap hardwareMap){

        /* Initialize Subsystems */
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = new Intake(hardwareMap);
        launcher = new Launcher(hardwareMap);
        slides = new Slides(hardwareMap);

        subsystems = Arrays.asList(
                intake,
                launcher,
                slides
        );

    }

    @Override
    public void update() {
        drive.update();
        for (Subsystem system: subsystems) {
            system.update();
        }
    }
}

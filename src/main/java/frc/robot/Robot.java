// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutoAimController.AutoAimLocation;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {}

    /**
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
     * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser make sure to add them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        m_autoSelected = m_chooser.getSelected();
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        System.out.println("Auto selected: " + m_autoSelected);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        switch (m_autoSelected) {
            case kCustomAuto:
                // Put custom auto code here
                break;
            case kDefaultAuto:
            default:
                // Put default auto code here
                break;
        }
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {}

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {}

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {}

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    Alliance alliance = Alliance.Red;

    XboxController joystick = new XboxController(0);

    PIDController xController = new PIDController(0, 0, 0);
    PIDController yController = new PIDController(0, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));
    TrajectoryConfig config = new TrajectoryConfig(4, 4);
    AutoAimController controller = new AutoAimController(xController, yController, thetaController, config);

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
        Pose2d startPose7 = new Pose2d(2.555, 4.75, Rotation2d.fromDegrees(270));
        Pose2d endPose7 = new Pose2d(2.323, 1.98, Rotation2d.fromDegrees(220));
        List<Translation2d> interPoints7 = new ArrayList<Translation2d>();
        interPoints7.add(new Translation2d(2.6, 2.48));

        // controller.setTarget(points, Rotation2d.fromDegrees(180));
        controller.setTarget(startPose7, interPoints7, endPose7, Rotation2d.fromDegrees(180));
    }

    private boolean wasPreviouslyPressed = false;
    private Pose2d desiredPose = new Pose2d(0, 0, new Rotation2d());
    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        desiredPose = new Pose2d(desiredPose.getX()+joystick.getRawAxis(0)*0.1, desiredPose.getY()-joystick.getRawAxis(1)*0.1, new Rotation2d(0));
        controller.getField().setRobotPose(desiredPose);

        if(joystick.getRawButton(1) && !wasPreviouslyPressed) {
            wasPreviouslyPressed = true;
            Pose2d currPose = controller.getField().getRobotPose();
            controller.setTarget(currPose, AutoAimLocation.MM, alliance);
            controller.enable(true);
        }
        if(!joystick.getRawButton(1)) {
            wasPreviouslyPressed = false;
            controller.enable(false);
        }
        controller.update();
    }
}

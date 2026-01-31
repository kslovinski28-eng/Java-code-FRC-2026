package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * REPLACES: Robot.cpp
 * 
 * Main robot class that handles robot lifecycle methods
 */
public class Robot extends TimedRobot {
    private static final String kAutoNameDefault = "Default";
    private static final String kAutoNameCustom = "My Auto";
    
    private RobotContainer robotContainer;
    private final SendableChooser<String> chooser = new SendableChooser<>();
    private String autoSelected;
    private double autoStartTime;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        chooser.addOption(kAutoNameDefault, kAutoNameDefault);
        chooser.addOption(kAutoNameCustom, kAutoNameCustom);
        chooser.setDefaultOption(kAutoNameDefault, kAutoNameDefault);
    }

    @Override
    public void robotPeriodic() {
        robotContainer.updateOdometry();
    }

    @Override
    public void teleopInit() {
        robotContainer.getNavX().zeroYaw();
    }

    @Override
    public void teleopPeriodic() {
        // Get joystick inputs
        double xSpeed = -robotContainer.getSwerveController()
            .getRawAxis(OperatorConstants.kForwardAxis);
        double ySpeed = robotContainer.getSwerveController()
            .getRawAxis(OperatorConstants.kStrafeAxis);
        double rot = robotContainer.getSwerveController()
            .getRawAxis(OperatorConstants.kRotationAxis);
        boolean fieldRelative = robotContainer.getSwerveController()
            .getRawButton(OperatorConstants.kFieldRelativeButton);

        // Speed scaling based on left trigger
        double triggerValue = robotContainer.getSwerveController()
            .getLeftTriggerAxis();
        double speedScale = 1.0;
        if (triggerValue > 0.2) {
            speedScale = 1.0 - ((triggerValue - 0.2) / 0.8) * 0.8;
        }

        xSpeed *= speedScale;
        ySpeed *= speedScale;
        rot *= speedScale;

        // Drive the robot
        robotContainer.drive(xSpeed, ySpeed, rot, fieldRelative);

        // Handle elevator mechanism
        double mechanismY = -robotContainer.getElevatorController()
            .getRawAxis(XboxController.Axis.kLeftY.value);
        robotContainer.setMechanismPosition(mechanismY);
    }

    @Override
    public void autonomousInit() {
        autoStartTime = Timer.getFPGATimestamp();
        autoSelected = chooser.getSelected();
    }

    @Override
    public void autonomousPeriodic() {
        double now = Timer.getFPGATimestamp();
        double elapsed = now - autoStartTime;

        if (elapsed < 2.0) {
            robotContainer.drive(0, 0.1, 0, true);
            var pose = robotContainer.getPose();
            if (pose.getY() > 0.5) {
                robotContainer.drive(0, 0, 0, true);
            }
        } else {
            robotContainer.drive(0, 0, 0, true);
        }
    }

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}
}

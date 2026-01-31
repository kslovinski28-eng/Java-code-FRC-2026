package frc.robot;

import com.studica.frc.AHRS;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.XboxController;

/**
 * REPLACES: RobotContainer.cpp
 * 
 * Container for robot subsystems and command configuration
 */
public class RobotContainer {
    // NavX gyro
    private final AHRS navx;

    // Controllers
    private final XboxController swerveController = new XboxController(0);
    private final XboxController elevatorController = new XboxController(1);

    // Swerve modules
    private final SwerveModule frontLeft = new SwerveModule(1, 2, 3, 0.0);
    private final SwerveModule frontRight = new SwerveModule(4, 5, 6, 0.0);
    private final SwerveModule backLeft = new SwerveModule(7, 8, 9, 0.0);
    private final SwerveModule backRight = new SwerveModule(10, 11, 12, 0.0);

    // Swerve kinematics - adjust these distances based on your robot
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(0.381, 0.381),   // Front left
        new Translation2d(0.381, -0.381),  // Front right
        new Translation2d(-0.381, 0.381),  // Back left
        new Translation2d(-0.381, -0.381)  // Back right
    );

    // Odometry
    private final SwerveDriveOdometry odometry;

    // Elevator motors
    private final SparkMax elevatorPivot = new SparkMax(13, MotorType.kBrushless);
    private final SparkMax elevatorPivot2 = new SparkMax(14, MotorType.kBrushless);

    public RobotContainer() {
        // Initialize NavX
        navx = new AHRS(AHRS.NavXComType.kMXP_SPI, (byte) 200);

        // Wait for NavX to connect
        int attempts = 0;
        final int maxAttempts = 250;
        while (!navx.isConnected() && attempts < maxAttempts) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            attempts++;
        }

        // Wait for NavX calibration
        if (navx.isConnected()) {
            while (navx.isCalibrating()) {
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
            navx.zeroYaw();
        }

        // Initialize odometry
        odometry = new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(0),
            new edu.wpi.first.math.kinematics.SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            },
            new Pose2d()
        );

        // Configure elevator leader motor
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        
        leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.1, 0.0, 0.0)
            .outputRange(-0.5, 0.5);

        elevatorPivot.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure elevator follower motor
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .follow(elevatorPivot.getDeviceId(), true);

        elevatorPivot2.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Drive the robot using field-centric or robot-centric control
     * 
     * @param xSpeed Forward/backward speed (-1.0 to 1.0)
     * @param ySpeed Left/right speed (-1.0 to 1.0)
     * @param rot Rotation speed (-1.0 to 1.0)
     * @param fieldRelative Whether to use field-centric control
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Apply deadband
        if (Math.abs(xSpeed) < OperatorConstants.kDeadband) xSpeed = 0.0;
        if (Math.abs(ySpeed) < OperatorConstants.kDeadband) ySpeed = 0.0;
        if (Math.abs(rot) < OperatorConstants.kDeadband) rot = 0.0;

        // Stop all modules if no input
        if (xSpeed == 0.0 && ySpeed == 0.0 && rot == 0.0) {
            frontLeft.setDesiredState(new edu.wpi.first.math.kinematics.SwerveModuleState(
                0.0, frontLeft.getState().angle));
            frontRight.setDesiredState(new edu.wpi.first.math.kinematics.SwerveModuleState(
                0.0, frontRight.getState().angle));
            backLeft.setDesiredState(new edu.wpi.first.math.kinematics.SwerveModuleState(
                0.0, backLeft.getState().angle));
            backRight.setDesiredState(new edu.wpi.first.math.kinematics.SwerveModuleState(
                0.0, backRight.getState().angle));
            return;
        }

        // Convert to chassis speeds
        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed * AutoConstants.kMaxSpeed,
                ySpeed * AutoConstants.kMaxSpeed,
                rot * AutoConstants.kMaxAngularSpeed,
                Rotation2d.fromDegrees(navx.getYaw())
            );
        } else {
            speeds = new ChassisSpeeds(
                xSpeed * AutoConstants.kMaxSpeed,
                ySpeed * AutoConstants.kMaxSpeed,
                rot * AutoConstants.kMaxAngularSpeed
            );
        }

        // Convert to swerve module states
        var states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, AutoConstants.kMaxSpeed);

        // Set desired states
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    /**
     * Update odometry with current module positions
     */
    public void updateOdometry() {
        odometry.update(
            Rotation2d.fromDegrees(navx.getYaw()),
            new edu.wpi.first.math.kinematics.SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );
    }

    /**
     * Set elevator mechanism position based on joystick input
     * 
     * @param joystickY Joystick Y-axis value (-1.0 to 1.0)
     */
    public void setMechanismPosition(double joystickY) {
        final double kMaxPosition = 10.0;
        final double kMinPosition = -10.0;
        final double kDeadband = 0.1;

        if (Math.abs(joystickY) < kDeadband) {
            return;
        }

        double targetPosition = joystickY * kMaxPosition;
        targetPosition = Math.max(kMinPosition, Math.min(kMaxPosition, targetPosition));

        elevatorPivot.getClosedLoopController().setReference(
            targetPosition, ControlType.kPosition
        );
    }

    /**
     * Get the current pose from odometry
     * 
     * @return Current robot pose
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    // Getters for controllers and sensors
    public XboxController getSwerveController() {
        return swerveController;
    }

    public XboxController getElevatorController() {
        return elevatorController;
    }

    public AHRS getNavX() {
        return navx;
    }
}

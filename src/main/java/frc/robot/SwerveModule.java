package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * REPLACES: SwerveModule.cpp
 * 
 * Represents a single swerve module with drive and steer motors
 */
public class SwerveModule {
    private static final double kWheelDiameter = 0.1016; // meters
    private static final double kGearRatio = 6.2;

    private final double angleOffset;
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder encoder;
    private final PIDController steerPID;

    private final DutyCycleOut driveControl = new DutyCycleOut(0);
    private final DutyCycleOut steerControl = new DutyCycleOut(0);

    /**
     * Construct a SwerveModule
     * 
     * @param driveID CAN ID of the drive motor
     * @param steerID CAN ID of the steer motor
     * @param encoderID CAN ID of the CANcoder
     * @param offset Angle offset in radians
     */
    public SwerveModule(int driveID, int steerID, int encoderID, double offset) {
        this.angleOffset = offset;
        this.driveMotor = new TalonFX(driveID);
        this.steerMotor = new TalonFX(steerID);
        this.encoder = new CANcoder(encoderID);
        this.steerPID = new PIDController(0.1, 0.0, 0.0002);

        // Configure drive motor
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        driveMotor.getConfigurator().apply(driveConfig);

        // Configure steer motor
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        steerMotor.getConfigurator().apply(steerConfig);

        // Enable continuous input for PID (wraps around -π to π)
        steerPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Set the desired state for the module
     * 
     * @param state Desired swerve module state
     */
    public void setDesiredState(SwerveModuleState state) {
        // Get current angle from encoder
        var encoderSignal = encoder.getAbsolutePosition();
        encoderSignal.refresh();
        double encoderTurns = encoderSignal.getValueAsDouble();

        // Convert to radians and adjust for offset
        double currentAngleRad = encoderTurns * 2.0 * Math.PI;
        double adjustedAngleRad = currentAngleRad - angleOffset;

        // Optimize the state to minimize rotation
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
            state, 
            new Rotation2d(adjustedAngleRad)
        );

        // Calculate drive output
        double driveOutput = optimizedState.speedMetersPerSecond / AutoConstants.kMaxSpeed;

        // Calculate steer output using PID
        double steerError = optimizedState.angle.getRadians() - adjustedAngleRad;
        double steerOutput = steerPID.calculate(steerError, 0.0);

        // Set motor outputs
        driveMotor.setControl(driveControl.withOutput(driveOutput));
        steerMotor.setControl(steerControl.withOutput(steerOutput));
    }

    /**
     * Get the current position of the module
     * 
     * @return Current swerve module position
     */
    public SwerveModulePosition getPosition() {
        // Get drive position in rotations
        double drivePosition = driveMotor.getPosition().getValueAsDouble();

        // Convert to meters
        double distanceMeters = drivePosition * (kWheelDiameter * Math.PI) / kGearRatio;

        // Get current angle
        double encoderTurns = encoder.getAbsolutePosition().getValueAsDouble();
        double currentAngleRad = encoderTurns * 2.0 * Math.PI;
        double adjustedAngleRad = currentAngleRad - angleOffset;

        return new SwerveModulePosition(distanceMeters, new Rotation2d(adjustedAngleRad));
    }

    /**
     * Get the current state of the module
     * 
     * @return Current swerve module state
     */
    public SwerveModuleState getState() {
        // Get drive velocity in rotations per second
        double driveVelocity = driveMotor.getVelocity().getValueAsDouble();

        // Convert to meters per second
        double speedMetersPerSecond = driveVelocity * (kWheelDiameter * Math.PI) / kGearRatio;

        // Get current angle
        double encoderTurns = encoder.getAbsolutePosition().getValueAsDouble();
        double currentAngleRad = encoderTurns * 2.0 * Math.PI;
        double adjustedAngleRad = currentAngleRad - angleOffset;

        return new SwerveModuleState(speedMetersPerSecond, new Rotation2d(adjustedAngleRad));
    }
}

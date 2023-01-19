package frc.robot.subsystems;

import java.util.zip.DeflaterOutputStream;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivetrainConstants;
import frc.thunder.swervelib.SdsModuleConfigurations;

public class NeoSwerveModule {

    private final double drivePositionConversionFactor = Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter()
            * SdsModuleConfigurations.MK4_L2.getDriveReduction();
    // TOOD look at this number when testing
    private final double steerPositionConversionFactor = 2.0 * Math.PI
            * SdsModuleConfigurations.MK4_L2.getSteerReduction();

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_turningEncoder;

    private final CANCoder m_canCoder;

    private final String define;

    public NeoSwerveModule(int driveMotorID, int azimuthMotorID, int canCoderID,
            double steerOffset, String define) {

        this.define = define;

        m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        m_driveMotor.restoreFactoryDefaults();
        m_driveMotor.enableVoltageCompensation(DrivetrainConstants.NOMINAL_VOLTAGE);
        m_driveMotor.setSmartCurrentLimit(DrivetrainConstants.DRIVE_CURRENT_LIMIT);
        m_driveMotor.setIdleMode(IdleMode.kBrake);

        m_turningMotor = new CANSparkMax(azimuthMotorID, MotorType.kBrushless);
        m_turningMotor.restoreFactoryDefaults();
        m_turningMotor.enableVoltageCompensation(DrivetrainConstants.NOMINAL_VOLTAGE);
        m_turningMotor.setSmartCurrentLimit(DrivetrainConstants.STEER_CURRENT_LIMIT);
        m_turningMotor.setIdleMode(IdleMode.kBrake);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turningEncoder = m_turningMotor.getEncoder();

        m_canCoder = new CANCoder(canCoderID);
        m_canCoder.configFactoryDefault();
        m_canCoder.configMagnetOffset(steerOffset);
        m_canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        m_canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        m_driveEncoder.setPositionConversionFactor(drivePositionConversionFactor);
        m_turningEncoder.setPositionConversionFactor(steerPositionConversionFactor);
        m_driveEncoder.setVelocityConversionFactor(drivePositionConversionFactor / 60);
        m_turningEncoder.setVelocityConversionFactor(steerPositionConversionFactor / 60);
    }

    private double getAbsolutePosition() {
        return Math.toRadians(m_canCoder.getAbsolutePosition());
    }

    /**
     * Returns the current state of the module.
     *
     * @return the current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity(),
                new Rotation2d(getAbsolutePosition()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveEncoder.getPosition(),
                new Rotation2d(getAbsolutePosition()));
    }

    /**
     * Retruns the current angle of the module in degrees
     * 
     * @return the current angle of the module
     */
    public double getSteerAngle() {
        return getState().angle.getDegrees();
    }

    /**
     * Returns the current drive velociry of the module
     * 
     * @return
     */
    public double getDriveVelocity() {
        return getState().speedMetersPerSecond;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees

        // SwerveModuleState state = SwerveModuleState.optimize(desiredState,
        //         new Rotation2d(getAbsolutePosition()));
 
        SwerveModuleState state = desiredState;

        // Calculate the drive output from the drive PID controller.
        double driveOutput = DrivetrainConstants.DRIVE_PID_CONTROLLER.calculate(
                m_driveEncoder.getVelocity(),
                state.speedMetersPerSecond);

        double driveFeedforward = DrivetrainConstants.DRIVE_FEED_FORWARD
                .calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller

        double turnOutput = DrivetrainConstants.AZIMUTH_PID_CONTROLLER.calculate(
                getAbsolutePosition(),
                state.angle.getRadians());

        double turnFeedForward = DrivetrainConstants.AZIMUTH_FEED_FORWARD
                .calculate(DrivetrainConstants.AZIMUTH_PID_CONTROLLER.getSetpoint().velocity);

        // double turnFeedforward = DrivetrainConstants.AZIMUTH_FEED_FORWARD
        // .calculate(DrivetrainConstants.AZIMUTH_PID_CONTROLLER.getSetpoint());

        // m_driveMotor.setVoltage(driveOutput + driveFeedforward);
        m_turningMotor.setVoltage(turnOutput + turnFeedForward);
    }

    public CANCoder getCancoder() {
        return m_canCoder;
    }
}

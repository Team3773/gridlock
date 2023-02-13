package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RobotMap;
import frc.thunder.config.NeoConfig;
import frc.thunder.config.SparkMaxPIDGains;
import frc.thunder.logging.DataLogger;

public class Arm extends SubsystemBase {
    private CANSparkMax motor;
    private SparkMaxPIDController controller;
    private SparkMaxAbsoluteEncoder encoder;
    private double OFFSET;
    private double targetAngle;

    public Arm() {
        if (Constants.isBlackout()) {
            // if blackout, use the blackout offset
            OFFSET = ArmConstants.ENCODER_OFFSET_BLACKOUT;
        } else {
            // otherwise, assume gridlock offset
            OFFSET = ArmConstants.ENCODER_OFFSET_GRIDLOCK;
        }

        motor = NeoConfig.createMotor(RobotMap.CAN.ARM_MOTOR, ArmConstants.MOTOR_INVERT,
                ArmConstants.CURRENT_LIMIT, Constants.VOLTAGE_COMP_VOLTAGE, ArmConstants.MOTOR_TYPE,
                ArmConstants.NEUTRAL_MODE);
        encoder = NeoConfig.createAbsoluteEncoder(motor, OFFSET);
        controller = NeoConfig.createPIDController(motor.getPIDController(), new SparkMaxPIDGains(
                ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, ArmConstants.kF), encoder);
        encoder.setPositionConversionFactor(ArmConstants.POSITION_CONVERSION_FACTOR);
        controller.setOutputRange(ArmConstants.MIN_POWER, ArmConstants.MAX_POWER);

        initLogging();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void initLogging() {
        DataLogger.addDataElement("Arm target angle", () -> targetAngle);
        DataLogger.addDataElement("Arm angle", () -> getAngle().getDegrees());
        DataLogger.addDataElement("Arm on target", () -> onTarget() ? 1 : 0);
        DataLogger.addDataElement("Arm motor temperature", () -> motor.getMotorTemperature());
        DataLogger.addDataElement("Arm Motor Controller Input Voltage", () -> motor.getBusVoltage());
        DataLogger.addDataElement("Arm Motor Controller Output (Amps)", () -> motor.getOutputCurrent());

    }

    /**
     * SetAngle: sets the angle of the arm to the angle in the given Rotation2d
     * object
     * 
     * @param angle a Rotation2d object containing the angle to set the arm to
     * 
     */
    public void setAngle(Rotation2d angle) {
        targetAngle = MathUtil.clamp(angle.getDegrees(), ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);
        controller.setReference(targetAngle, CANSparkMax.ControlType.kPosition);
    }

    /**
     * GetAngle
     * 
     * @return the angle of the arm as a Rotation2d object
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(encoder.getPosition());
    }

    /**
     * SetPower: sets the percent power of the arm motor
     * 
     * @param power the percent power to set the arm motor to
     */
    public void setPower(double power) {
        motor.set(MathUtil.clamp(power, ArmConstants.MIN_POWER, ArmConstants.MAX_POWER));
    }

    /**
     * Stop: sets the arm motor to 0% power
     */
    public void stop() {
        setPower(0d);
    }

    /**
     * onTarget
     * 
     * @return true if the arm is within the tolerance of the target angle
     */
    public boolean onTarget() {
        return Math.abs(getAngle().getDegrees() - targetAngle) < ArmConstants.TOLERANCE;
    }
}

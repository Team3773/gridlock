package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RobotMap;
import frc.thunder.config.NeoConfig;
import frc.thunder.config.SparkMaxPIDGains;
import frc.thunder.logging.DataLogger;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.tuning.PIDDashboardTuner;

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
        encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        controller = NeoConfig.createPIDController(motor.getPIDController(),
                new SparkMaxPIDGains(ArmConstants.DOWN_kP, ArmConstants.DOWN_kI,
                        ArmConstants.DOWN_kD, ArmConstants.DOWN_kF),
                new SparkMaxPIDGains(ArmConstants.UP_kP, ArmConstants.UP_kI, ArmConstants.UP_kD,
                        ArmConstants.UP_kF),
                encoder);
        encoder.setPositionConversionFactor(360);
        // encoder.setZeroOffset(-194);
        controller.setOutputRange(ArmConstants.MIN_POWER, ArmConstants.MAX_POWER);
        motor.setClosedLoopRampRate(2);


        // PIDDashboardTuner tuner = new PIDDashboardTuner("Arm", controller);

        initLogging();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void initLogging() {
        DataLogger.addDataElement("Arm target angle", () -> targetAngle);
        DataLogger.addDataElement("Arm angle", () -> getAngle().getDegrees());
        DataLogger.addDataElement("Arm on target", () -> onTarget() ? 1 : 0);
        DataLogger.addDataElement("Arm motor temperature", () -> motor.getMotorTemperature());
        DataLogger.addDataElement("Arm Motor Controller Input Voltage",
                () -> motor.getBusVoltage());
        DataLogger.addDataElement("Arm Motor Controller Output (Amps)",
                () -> motor.getOutputCurrent());

    }

    /**
     * SetAngle: sets the angle of the arm to the angle in the given Rotation2d object
     * 
     * @param angle a Rotation2d object containing the angle to set the arm to
     * 
     */
    public void setAngle(Rotation2d angle) {
        targetAngle =
                MathUtil.clamp(angle.getDegrees(), ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);

        // LightningShuffleboard.setDouble("Arm", "target agle", targetAngle);
        // if(targetAngle - getAngle().getDegrees() > 5) {
        if (targetAngle - getAngle().getDegrees() > 2) {
            controller.setReference(targetAngle + OFFSET, CANSparkMax.ControlType.kPosition, 1);
        } else {
            controller.setReference(targetAngle + OFFSET, CANSparkMax.ControlType.kPosition, 0);
        }
        // } else {
        // controller.setReference(, CANSparkMax.ControlType.kPosition, 0);
        // }

        // controller.setReference(targetAngle + OFFSET, CANSparkMax.ControlType.kPosition, 1);

    }

    /**
     * GetAngle
     * 
     * @return the angle of the arm as a Rotation2d object
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(encoder.getPosition() - OFFSET);
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
     * getBottomLimitSwitch
     * 
     * @return true if the bottom limit switch is pressed
     */
    public boolean getReverseLimitSwitch() {
        return motor.getReverseLimitSwitch(ArmConstants.BOTTOM_LIMIT_SWITCH_TYPE).isPressed();
    }

    /**
     * getTopLimitSwitch
     * 
     * @return true if the top limit switch is pressed
     */
    public boolean getForwardLimitSwitch() {
        return motor.getForwardLimitSwitch(ArmConstants.TOP_LIMIT_SWITCH_TYPE).isPressed();
    }

    /**
     * onTarget
     * 
     * @return true if the arm is within the tolerance of the target angle
     */
    public boolean onTarget() {
        return Math.abs(getAngle().getDegrees() - targetAngle) < ArmConstants.TOLERANCE;
    }

    public boolean isReachable(Rotation2d angle) {
        return angle.getDegrees() >= ArmConstants.MIN_ANGLE
                && angle.getDegrees() <= ArmConstants.MAX_ANGLE;
    }

    @Override
    public void periodic() {
        LightningShuffleboard.setBool("Arm", "fwd Limit", getForwardLimitSwitch());
        LightningShuffleboard.setBool("Arm", "rev Limit", getReverseLimitSwitch());
        LightningShuffleboard.setDouble("Arm", "absolute encoder", getAngle().getDegrees());

        setAngle(Rotation2d.fromDegrees(LightningShuffleboard.getDouble("Arm", "setpoint", -90)));

        double kf = LightningShuffleboard.getDouble("Arm", "kF", ArmConstants.UP_kF);
        // double kf = ArmConstants.ARM_UP_KF_MAP.get(getAngle().getDegrees());
        double kp = LightningShuffleboard.getDouble("Arm", "up kP", ArmConstants.UP_kP);
        controller.setP(kp, 1);
        controller.setP(kp, 0);
        controller.setFF(kf, 1);
        controller.setFF(kf, 0);

        LightningShuffleboard.setDouble("Arm", "current FF", kf);


        // controller.setFF(LightningShuffleboard.getDouble("Arm", "down kF", ArmConstants.DOWN_kF),
        // 0);
        motor.setClosedLoopRampRate(LightningShuffleboard.getDouble("Arm", "ramp rate", 2));

        LightningShuffleboard.setDouble("Arm", "curr speed", motor.get());
    }
}

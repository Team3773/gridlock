package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.RobotMap;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;

/**
 * The servo subsystem
 */
public class ServoTurn extends SubsystemBase {
    private Servo servo = new Servo(RobotMap.PWM.SERVO);
    private double position = AutonomousConstants.SERVO_STOW;

    private LightningShuffleboardPeriodic periodicShuffleboard;

    public ServoTurn() {
        // Initialize the shuffleboard values and start logging data
        initializeShuffleboard();
    }

    // Initializes the shuffleboard values and starts logging data   
    @SuppressWarnings("unchecked")
    private void initializeShuffleboard() {
        periodicShuffleboard = new LightningShuffleboardPeriodic("ServoTurn", 2, new Pair<String, Object>("Servo Position", (DoubleSupplier) () -> position));
    }

    /**
     * Turns the servo to the specified position
     * 
     * @param position the position to turn the servo to, between 0 and 1
     */

    public void turnServo(double position) {
        this.position = position;
        servo.set(position);
    }

    @Override
    public void periodic() {
        if (DriverStation.isAutonomous()) {
            periodicShuffleboard.loop();
        }
    }
}

package frc.robot;

import java.nio.file.Path;
import java.nio.file.Paths;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.thunder.math.InterpolationMap;
import frc.thunder.pathplanner.com.pathplanner.lib.auto.PIDConstants;
import frc.thunder.swervelib.SdsModuleConfigurations;

/**
 * Class to hold all of the constants for the robot
 */
public final class Constants {

    // Spark max voltage compensation
    public static final double VOLTAGE_COMPENSATION = 12d;

    // Path to the blackout directory
    public static final Path BLACKOUT_PATH = Paths.get("home/lvuser/blackout");

    // Check if we're on blackout
    public static final boolean isBlackout() {
        return BLACKOUT_PATH.toFile().exists();
    }

    // Check if we're on gridlock
    public static final boolean isGridlock() {
        return !isBlackout();
    }

    public static final double COMP_LOG_PERIOD = .33;

    // Constants for xbox controlers
    public static final class XboxControllerConstants {
        // Ports for the controllers
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int COPILOT_CONTROLLER_PORT = 1;

        // Deadband, min, and max power for the controllers
        public static final double DEADBAND = 0.1d;
        public static final double MIN_POWER = 0d;
        public static final double MAX_POWER = 1d;

    }

    public final static class OperationConstants {
        public static final int armExtendMotorChannel = 9;
        public static final int armRotateMotorChannel = 10;
        public static final int elevatorMotorChannel = 11;
        public static final int clawMotorChannel = 12;
  
        public static final double kArmRotateGearRatio = 1/977;
        public static final double kArmExtendGearRatio = 1/343; // EDUCATED GUESS
        public static final double kElevatorGearRatio = 1/36;
        public static final double kClawGearRatio = 1/196;
  
        public static final double kArmRotateEncoderRot2Meter = 1.0 / 4096.0 * 2 * Math.PI * kArmRotateGearRatio;
        public static final double kArmExtendEncoderRot2Meter = 2 * Math.PI * kArmExtendGearRatio;
        public static final double kElevatorEncoderRot2Meter = 2 * Math.PI * kElevatorGearRatio;
        public static final double kClawEncoderRot2Meter = 2 * Math.PI * kClawGearRatio;
  
        // CHANGE TO ACTUAL SETPOINTS
        public static final double kArmRotateSetpoint = 0;
        public static final double kArmExtendSetpoint = 0;
        public static final double kElevatorSetpoint = 0;
        public static final double kClawSetpoint = 0;
  
        // DAMPNERS
        public static final double kArmRotateDampner = .8;
        public static final double kArmExtendDampner = .8;
        public static final double kSwerveDeadband = 0.1;
        public static final double setpointDeadband = 5;
  
        // CHARGE STATION
        public static final double kBeam_Balance_Goal_Degrees = 0.0;
  
        // CHANGE VALUES BASED ON ENCODER READINGS
        public static final double kElevatorBottomPosition = -129.45;
        public static final double kElevatorTopPosition = 0;
  
        public static final double kClawStopGrabPoint = 0;
        
        public static final double kTopArmRotatePoint = 0;
        public static final double kBottomArmRotatePoint = 0;
  
        public static final double kTopArmExtendPoint = 0;
        public static final double kBottomArmExtendPoint = 0;
  
        // EXTERNAL ENCODER PORTS
        public static final int karmExtendEncoderA = 0;
        public static final int karmExtendEncoderB = 1;
        public static final int karmRotateEncoderA = 2;
        public static final int karmRotateEncoderB = 3;
  
        public static final int limitSwitchPort = 4;
      }

    // Constants for our system tests    
    public static final class SystemTestConstants {
        // Drive Test Variables
        public static final int DEGREES_INTERVAL_INCREASE = 30;
        public static final int ANGLE_DEAD_ZONE = 3;
        public static final int MAX_ROTATIONS_PER_DIRECTION = 2;
    }

    // COnstants for our drivetrain
    public static final class DrivetrainConstants {
        // Our drivetrain and track width
        // TODO: remeasure these

        // public static final double kTrackWidth = Units.inchesToMeters(28);
        // // Distance between right and left wheels
        // public static final double kWheelBase = Units.inchesToMeters(32);
        // // Distance between front and back wheels
        // public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        //     new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        //     new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        //     new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        //     new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(28);
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(32);

        // Module resting/default angles 
        public static final double FRONT_LEFT_RESTING_ANGLE = Math.toRadians(-45d); //flip these
        public static final double FRONT_RIGHT_RESTING_ANGLE = Math.toRadians(45d);
        public static final double BACK_LEFT_RESTING_ANGLE = Math.toRadians(45d);
        public static final double BACK_RIGHT_RESTING_ANGLE = Math.toRadians(-45d);

        // Our max voltage, velocity, angular velocity, and angular acceleration
        public static final double MAX_VOLTAGE = 12;
        // public static final double MAX_VELOCITY_METERS_PER_SECOND = 5676.0 / 60.0 * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5; //12
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 4* Math.PI;//MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND = 3;//MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 2 * Math.PI / 5;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND);

        // Module configuration constants
        public static final int DRIVE_CURRENT_LIMIT = 20; //40;
        public static final int STEER_CURRENT_LIMIT = 20; //30;
        public static final double NOMINAL_VOLTAGE = 12d;

        public static final double LOG_PERIOD = 0.18;

        public static final double SLOW_MODE_TRANSLATIONAL_MULT = 0.4;
        public static final double SLOW_MODE_ROTATIONAL_MULT = 0.4;

        // Pigeon heading offset 
        // public static final Rotation2d HEADING_OFFSET = Rotation2d.fromDegrees(90);

        // Standard dev for robot pose
        public static final Matrix<N3, N1> STANDARD_DEV_POSE_MATRIX = VecBuilder.fill(1, 1, 0.0368); // (0.3313838876, 0.2642363651, 0.03681853519);

        // Gains vaules for PIDControllers
        public static final class Gains {
            public static final double kP = 0.05;//0.05;//0.15;// .116d; (ENABLES DRIVE)
            public static final double kI = 0d;
            public static final double kD = 0d;

            public static final double kF = 0.225;//0.225;// 229d;
        }

        // Gains vaules for theta PIDControllers
        public static final class ThetaGains {
            public static final double kP = 0;
            public static final double kI = 0d;
            public static final double kD = 0d;

        }

        // PID gains for our heading compensation
        public static final class HeadingGains {
            public static final double kP = 0.005d;
            public static final double kI = 0d;
            public static final double kD = 0d;
        }

        // Steer offsets for our modules
        public static final class Offsets {
            // Gridlocks steer offsets
            public static final class Gridlock {
                public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(221.221); // 9 89.033 BECOME 10
                public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(142.207 + 4); // 10 117.861 BECOME 9 120.322
                public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(120.322 + 34); // 11 111.445 BECOME 12 142.207
                public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(248.467 - 14); // 12 61.172 BECOME
            }

            // Blackouts steer offsets
            public static final class Blackout {
                public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(118.213);
                public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(92.021);
                public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(49.834);
                public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(105.117);
            }
        }
    }

    // Constants for our elevator
    public static final class LimelightConstants {
        public static final String FRONT_NAME = "limelight-front";
        public static final String BACK_NAME = "limelight-back";
        public static final Pose3d FRONT_POSE = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
        public static final Pose3d BACK_POSE = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

    }

    // RobotMap Constants
    public static final class RobotMap {
        // CAN IDs
        public static final class CAN {
            // Pigeon IMU ID
            // public static final int PIGEON_ID = 23;
            // Power distrobution hub ID
            // public static final int PDH = 21;

            // Front left CanIDs
            public static final int FRONT_LEFT_DRIVE_MOTOR = 2;
            public static final int FRONT_LEFT_AZIMUTH_MOTOR = 1;
            public static final int FRONT_LEFT_CANCODER = 9;
            // Front right CanIDs
            public static final int FRONT_RIGHT_DRIVE_MOTOR = 3;
            public static final int FRONT_RIGHT_AZIMUTH_MOTOR = 4;
            public static final int FRONT_RIGHT_CANCODER = 10;
            // Back right CanIDs
            public static final int BACK_RIGHT_DRIVE_MOTOR = 5;
            public static final int BACK_RIGHT_AZIMUTH_MOTOR = 6;
            public static final int BACK_RIGHT_CANCODER = 12;
            // Back left CanIDs
            public static final int BACK_LEFT_DRIVE_MOTOR = 8;
            public static final int BACK_LEFT_AZIMUTH_MOTOR = 7;
            public static final int BACK_LEFT_CANCODER = 11;

            // public static final int kFrontLeftDriveMotorPort = 2;
            // public static final int kFrontRightDriveMotorPort = 3;
            // public static final int kBackRightDriveMotorPort = 5;
            // public static final int kBackLeftDriveMotorPort = 8;
    
            // public static final int kFrontLeftTurningMotorPort = 1;
            // public static final int kFrontRightTurningMotorPort = 4;
            // public static final int kBackRightTurningMotorPort = 6;
            // public static final int kBackLeftTurningMotorPort = 7;
        }

        public static final class PWM {
            public static final int SERVO = 0;
        }

        public static final class i2c { //Lowercase to avoid conflict with wpilib's I2C class
            public static final I2C.Port COLOR_SENSOR = I2C.Port.kMXP;
        }
    }

    // Constants used for auto balancing
    public static final class AutoBalanceConstants {
        // Magnitude for being balance
        public static final double BALANCED_MAGNITUDE = 2.5;

        // Upper and lower magnitude thresholds for checking if we are on the charge station at all
        public static final double UPPER_MAGNITUDE_THRESHOLD = 11;
        public static final double LOWER_MAGNITUDE_THRESHOLD = 7; // TODO RESET Value if possible was 2.5 If mag jumps during Auton
        // Min and max speeds for our auto balance
        public static final double MIN_SPEED_THRESHOLD = 0.35;
        public static final double MAX_SPEED_THRESHOLD = 3;

        // Delay time for our auto balance after falling
        public static final double DELAY_TIME = 1.5;
        public static final double LOG_PERIOD = 0.2;

        // Target X position for the middle of the charge station
        public static final double TARGET_X = 3.93;

        // Gains for our auto balance
        public static final double kP = 2;//2;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    // Constants for vision
    public static final class VisionConstants {
        // Represents camera FOV from center to edge
        public static final double HORIZ_CAMERA_FOV = 29.8d;

        // Arbitrary value for how close the robot needs to be to the target (in angles)
        public static final double HORIZ_DEGREE_TOLERANCE = 3d;

        // Standard deviation for vision, heading is 1000 becuase were using pigeon, so i dont want to use vision heading
        public static final Matrix<N3, N1> STANDARD_DEV_VISION_MATRIX = VecBuilder.fill(2, 1.4, 1000); //(1.195384707229739, 0.7850610924749237, 2.2025094640913276);
    }

    // Constants for the lift

    // Constants for autonomous
    public static final class AutonomousConstants {
        // Path planner PIDConstants
        public static final PIDConstants DRIVE_PID_CONSTANTS = new PIDConstants(0.05, 0, 0); // Drive velocity PID 10.5
        public static final PIDConstants THETA_PID_CONSTANTS = new PIDConstants(0.1, 0, 0); // Rotation PID 7
        public static final PIDConstants POSE_PID_CONSTANTS = new PIDConstants(0, 0, 0); // X and Y position PID
        
        // Max velocity and acceleration for the path planner
        public static final double MAX_VELOCITY = 1;
        public static final double MAX_ACCELERATION = .25;
        public static final double SERVO_DEPLOY = 0.4;
        public static final double SERVO_STOW = 0d;

    }

    //Constants for autoAlign
    public static final class AutoAlignConstants {
        // PID constants for auto align
        public static final PIDConstants AUTO_ALIGN_PID_CONSTANTS = new PIDConstants(0.009, 0, 0);

        // Tolerance for auto align
        public static final double TOLERANCE = 1d;

        //Offset limelight off center
        public static final double LIMELGHT_OFFSET_BLACKOUT = -11.5;
        public static final double LIMELGHT_OFFSET_GRIDLOCK = -12.2;
    }
}

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import frc.thunder.swervelib.Mk4ModuleConfiguration;
import frc.thunder.swervelib.Mk4iSwerveModuleHelper;
import frc.thunder.swervelib.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.Offsets;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.DrivetrainConstants.Gains;
import frc.robot.Constants.DrivetrainConstants.HeadingGains;
import frc.thunder.config.SparkMaxPIDGains;
import frc.thunder.limelightlib.LimelightHelpers;
import frc.thunder.pathplanner.com.pathplanner.lib.PathPoint;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.shuffleboard.LightningShuffleboardPeriodic;

/**
 * The drivetrain subsystem
 */
public class Drivetrain extends SubsystemBase {

    // Creates our swerve kinematics using the robots track width and wheel base

    // public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
    //     new Translation2d(kWheelBase / 2, kTrackWidth / 2),
    //     new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
    //     new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
    //     new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                // Front left (9)
                new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Front right (10)
                new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Back left (11)
                new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Back right (12)
                new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

    // Creating new pigeon2 IMU
    // private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(RobotMap.CAN.PIGEON_ID);
    private final AHRS pigeon = new AHRS(SPI.Port.kMXP);

    // Creating our list of module states and module positions
    private SwerveModuleState[] states = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
    private SwerveModulePosition[] modulePositions = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};

    // Creating new pose, odometry, cahssis speeds
    private Pose2d pose = new Pose2d();
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    // Creating our modules
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    // Module steer offsets
    private double FRONT_LEFT_STEER_OFFSET = Offsets.Gridlock.FRONT_LEFT_STEER_OFFSET;
    private double BACK_LEFT_STEER_OFFSET = Offsets.Gridlock.BACK_LEFT_STEER_OFFSET;
    private double FRONT_RIGHT_STEER_OFFSET = Offsets.Gridlock.FRONT_RIGHT_STEER_OFFSET;
    private double BACK_RIGHT_STEER_OFFSET = Offsets.Gridlock.BACK_RIGHT_STEER_OFFSET;

    // Swerve pose esitmator for odometry
    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getYaw2d(), modulePositions);      
        
    // Creates our drivetrain shuffleboard tab for displaying module data and a periodic shuffleboard for data that doesn't need constant updates
    private ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    private LightningShuffleboardPeriodic periodicShuffleboard;
    private final Mk4ModuleConfiguration swerveConfiguration = new Mk4ModuleConfiguration();
    private final Mk4ModuleConfiguration blSwerveConfiguration = new Mk4ModuleConfiguration();

    // PIDController for heading compenstation
    private final PIDController headingController = new PIDController(HeadingGains.kP, HeadingGains.kI, HeadingGains.kD);

    private boolean flipFL = false;
    private boolean flipFR = false;
    private boolean flipBR = false;
    private boolean flipBL = false;

    // Heading compenstaion variables
    // private boolean updatedHeading = false;
    // private double lastGoodheading = 0d;

    // Chassis speeds for the robot
    private ChassisSpeeds outputChassisSpeeds = new ChassisSpeeds();

    public Drivetrain() {
        /**
         * Creates a new Drivetrain.
         * 
         * @param vision the vision subsystem
         */

        if (Constants.isBlackout()) {
            FRONT_LEFT_STEER_OFFSET = Offsets.Blackout.FRONT_LEFT_STEER_OFFSET;
            FRONT_RIGHT_STEER_OFFSET = Offsets.Blackout.FRONT_RIGHT_STEER_OFFSET;
            BACK_LEFT_STEER_OFFSET = Offsets.Blackout.BACK_LEFT_STEER_OFFSET;
            BACK_RIGHT_STEER_OFFSET = Offsets.Blackout.BACK_RIGHT_STEER_OFFSET;
        }

        // Set our neo module configurations using drive current, steer current, and
        // voltage
        swerveConfiguration.setDriveCurrentLimit(DrivetrainConstants.DRIVE_CURRENT_LIMIT);
        swerveConfiguration.setSteerCurrentLimit(DrivetrainConstants.STEER_CURRENT_LIMIT);
        swerveConfiguration.setNominalVoltage(DrivetrainConstants.NOMINAL_VOLTAGE);
        swerveConfiguration.setDrivePIDGains(new SparkMaxPIDGains(Gains.kP, Gains.kI, Gains.kD, Gains.kF));

        blSwerveConfiguration.setDriveCurrentLimit(DrivetrainConstants.DRIVE_CURRENT_LIMIT);
        blSwerveConfiguration.setSteerCurrentLimit(DrivetrainConstants.STEER_CURRENT_LIMIT);
        blSwerveConfiguration.setNominalVoltage(DrivetrainConstants.NOMINAL_VOLTAGE);
        blSwerveConfiguration.setDrivePIDGains(new SparkMaxPIDGains(Gains.kP, Gains.kI, Gains.kD, Gains.kF));

        // Making front left module
        frontLeftModule = Mk4iSwerveModuleHelper.createNeo(tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0), swerveConfiguration,
                Mk4iSwerveModuleHelper.GearRatio.L2, RobotMap.CAN.FRONT_LEFT_DRIVE_MOTOR, RobotMap.CAN.FRONT_LEFT_AZIMUTH_MOTOR, RobotMap.CAN.FRONT_LEFT_CANCODER, FRONT_LEFT_STEER_OFFSET);

        // Making front right module
        frontRightModule = Mk4iSwerveModuleHelper.createNeo(tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0), swerveConfiguration,
                Mk4iSwerveModuleHelper.GearRatio.L2, RobotMap.CAN.FRONT_RIGHT_DRIVE_MOTOR, RobotMap.CAN.FRONT_RIGHT_AZIMUTH_MOTOR, RobotMap.CAN.FRONT_RIGHT_CANCODER, FRONT_RIGHT_STEER_OFFSET);

        // Making backleft module
        backLeftModule = Mk4iSwerveModuleHelper.createNeo(tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0), blSwerveConfiguration,
                Mk4iSwerveModuleHelper.GearRatio.L2, RobotMap.CAN.BACK_LEFT_DRIVE_MOTOR, RobotMap.CAN.BACK_LEFT_AZIMUTH_MOTOR, RobotMap.CAN.BACK_LEFT_CANCODER, BACK_LEFT_STEER_OFFSET);

        // Making back right module
        backRightModule = Mk4iSwerveModuleHelper.createNeo(tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0), swerveConfiguration,
                Mk4iSwerveModuleHelper.GearRatio.L2, RobotMap.CAN.BACK_RIGHT_DRIVE_MOTOR, RobotMap.CAN.BACK_RIGHT_AZIMUTH_MOTOR, RobotMap.CAN.BACK_RIGHT_CANCODER, BACK_RIGHT_STEER_OFFSET);

        // Setting states of the modules
        updateOdometry();
        updateDriveStates(states);

        // Initialize the shuffleboard values and start logging data
        initializeShuffleboard();

        // Zero our gyro
        zeroHeading();

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        
        // Update our odometry
        updateOdometry();

        periodicShuffleboard.loop();
    }

    /**
     * Method to return the heading controller
     * 
     * @return the heading PIDController
     */
    public PIDController getHeadingController() {
        return headingController;
    }

    // Checks if all the modules states speeds are 0
    // private boolean checkModuleStates() {
    //     return Math.abs(states[0].speedMetersPerSecond + states[1].speedMetersPerSecond + states[2].speedMetersPerSecond + states[3].speedMetersPerSecond) == 0;
    // }

    /**
     * This takes chassis speeds and converts them to module states and then sets states.
     * 
     * @param chassisSpeeds the chassis speeds to convert to module states
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        outputChassisSpeeds = chassisSpeeds;

        // If we havent updated the heading last known good heading, update it
        // if (!updatedHeading) {
        //     lastGoodheading = pose.getRotation().getDegrees();
        //     updatedHeading = true;
        // }

        // If we are not command a rotation for the robot and are moudle states are not 0 comensate for any rotational drift
        // if (chassisSpeeds.omegaRadiansPerSecond == 0 && !checkModuleStates()) {
        //     outputChassisSpeeds.omegaRadiansPerSecond =
        //             headingController.calculate(pose.getRotation().getDegrees(), lastGoodheading);
        // } else {
        //     // If we are command a rotation then our updated heading is no longer valid so this will help reset it 
        //     updatedHeading = false;
        // }

        // If were not commanding any thing to the motors, make sure our states speeds are 0
        if (states != null && chassisSpeeds.vxMetersPerSecond == 0 && chassisSpeeds.vyMetersPerSecond == 0 && chassisSpeeds.omegaRadiansPerSecond == 0) {
            states[0].speedMetersPerSecond = 0;
            states[1].speedMetersPerSecond = 0;
            states[2].speedMetersPerSecond = 0;
            states[3].speedMetersPerSecond = 0;

        } else {
            // If not set the states to the commanded chassis speeds
            states = kinematics.toSwerveModuleStates(outputChassisSpeeds);
        }
        
        if(flipFL) {
            states[0].speedMetersPerSecond *= -1;
        }
        if(flipFR) {
            states[1].speedMetersPerSecond *= -1;
        }
        if(flipBL) {
            states[2].speedMetersPerSecond *= -1;
        }
        if(flipBR) {
            states[3].speedMetersPerSecond *= -1; 
        }

        // Sets the states to the modules
        setStates(states);
    }

    public void flipFL() {
        flipFL = !flipFL;
    }
    public void flipFR() {
        flipFR = !flipFR;
    }
    public void flipBL() {
        flipBL = !flipBL;
    }
    public void flipBR() {
        flipBR = !flipBR;
    }

    /**
     * This takes a list of module states and sets them to the modules.
     * 
     * @param states the list of module states to set
     */
    public void updateDriveStates(SwerveModuleState[] states) {
        if (states != null) {
            SwerveModuleState frontLeftState = states[0];
            SwerveModuleState frontRightState = states[1];
            SwerveModuleState backLeftState = states[2];
            SwerveModuleState backRightState = states[3];

            // Normalize the wheel speeds if the magnitude of any wheel is greater than max velocity
            SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);

            // Sets the states to the modules
            frontLeftModule.set(frontLeftState.speedMetersPerSecond, frontLeftState.angle.getRadians());
            frontRightModule.set(frontRightState.speedMetersPerSecond, frontRightState.angle.getRadians());
            backLeftModule.set(backLeftState.speedMetersPerSecond, backLeftState.angle.getRadians());
            backRightModule.set(backRightState.speedMetersPerSecond, backRightState.angle.getRadians());
        }
    }

    /**
     * Updates odometry using the current yaw and module states.
     */
    public void updateOdometry() {
        updateModulePositions();
        pose = odometry.update(getYaw2d(), modulePositions);
        // pose = estimator.update(getYaw2d(), modulePositions);
        // if (visionFront.hasVision()) {
        //     double[] visionPose = LimelightHelpers.getBotPose_wpiBlue(visionFront.limelightName);
        //     Pose2d visionPose2d = new Pose2d(visionPose[0], visionPose[1], Rotation2d.fromDegrees(visionPose[2]));
        //     if (visionPose != null) {
        //         estimator.addVisionMeasurement(visionPose2d, Timer.getFPGATimestamp() - visionFront.getLatencyBotPoseBlue());
        //     }
        // } else if (visionBack.hasVision()) {
        //     double[] visionPose = LimelightHelpers.getBotPose_wpiBlue(visionBack.limelightName);
        //     Pose2d visionPose2d = new Pose2d(visionPose[0], visionPose[1], Rotation2d.fromDegrees(visionPose[2]));
        //     if (visionPose != null) {
        //         estimator.addVisionMeasurement(visionPose2d, Timer.getFPGATimestamp() - visionBack.getLatencyBotPoseBlue());
        //     }
        // }

    }

    /**
     * 2 Method to set states of modules.
     */
    public void setStates(SwerveModuleState[] newStates) {
        states = newStates;
        updateOdometry();
        updateDriveStates(states);

    }

    /**
     * Updates the module positions array to the current positions of each module
     * Relative encoder for Neo
     */
    public void updateModulePositions() {
        modulePositions[0] = frontLeftModule.getPosition();
        modulePositions[1] = frontRightModule.getPosition();
        modulePositions[2] = backLeftModule.getPosition();
        modulePositions[3] = backRightModule.getPosition();
    }

    // Method to start sending values to the dashboard and start logging
    @SuppressWarnings("unchecked")
    private void initializeShuffleboard() {
        periodicShuffleboard = new LightningShuffleboardPeriodic("Drivetrain", DrivetrainConstants.LOG_PERIOD, 
                // new Pair<String, Object>("Front left drive voltage", (DoubleSupplier) () -> frontLeftModule.getDriveVoltage()),
                // new Pair<String, Object>("Front right drive voltage", (DoubleSupplier) () -> frontRightModule.getDriveVoltage()),
                // new Pair<String, Object>("Back left drive voltage", (DoubleSupplier) () -> backLeftModule.getDriveVoltage()),
                // new Pair<String, Object>("Back right drive voltage", (DoubleSupplier) () -> backRightModule.getDriveVoltage()),
                // new Pair<String, Object>("Front left target angle", (DoubleSupplier) () -> states[0].angle.getDegrees()),
                // new Pair<String, Object>("Front right target angle", (DoubleSupplier) () -> states[1].angle.getDegrees()),
                // new Pair<String, Object>("Back left target angle", (DoubleSupplier) () -> states[2].angle.getDegrees()),
                // new Pair<String, Object>("Back right target angle", (DoubleSupplier) () -> states[3].angle.getDegrees()),
                new Pair<String, Object>("fl target velocity", (DoubleSupplier) () -> states[0].speedMetersPerSecond), // TODO: cpare these to acutal speeds
                new Pair<String, Object>("fr target velocity", (DoubleSupplier) () -> states[1].speedMetersPerSecond),
                new Pair<String, Object>("bl target velocity", (DoubleSupplier) () -> states[2].speedMetersPerSecond),
                new Pair<String, Object>("br target velocity", (DoubleSupplier) () -> states[3].speedMetersPerSecond),
                new Pair<String, Object>("fl target angle", (DoubleSupplier) () -> states[0].angle.getDegrees()),
                new Pair<String, Object>("fr target angle", (DoubleSupplier) () -> states[1].angle.getDegrees()),
                new Pair<String, Object>("bl target angle", (DoubleSupplier) () -> states[2].angle.getDegrees()),
                new Pair<String, Object>("br target angle", (DoubleSupplier) () -> states[3].angle.getDegrees()),
                new Pair<String, Object>("Odometry Heading", (DoubleSupplier) () -> getHeading().getDegrees()), 
                new Pair<String, Object>("roll", (DoubleSupplier) () -> pigeon.getRoll()), 
                new Pair<String, Object>("pitch", (DoubleSupplier) () -> pigeon.getPitch()), 
                new Pair<String, Object>("odo X", (DoubleSupplier) () -> pose.getX()),
                new Pair<String, Object>("odo Y", (DoubleSupplier) () -> pose.getY()), 
                new Pair<String, Object>("odo Z", (DoubleSupplier) () -> pose.getRotation().getDegrees()),
                new Pair<String, Object>("fl module position", (DoubleSupplier) () -> modulePositions[0].distanceMeters),
                new Pair<String, Object>("fr module position", (DoubleSupplier) () -> modulePositions[1].distanceMeters),
                new Pair<String, Object>("bl module position", (DoubleSupplier) () -> modulePositions[2].distanceMeters),
                new Pair<String, Object>("br module position", (DoubleSupplier) () -> modulePositions[3].distanceMeters),
                new Pair<String, Object>("odo Pose", (Supplier<double[]>) () -> new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()}));

        // // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Front left angle", () -> frontLeftModule.getSteerAngle());
        // // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Front right angle", () -> frontRightModule.getSteerAngle());
        // // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Back left angle", () -> backLeftModule.getSteerAngle());
        // // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Back right angle", () -> backRightModule.getSteerAngle());

        // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Front left drive velocity", () -> frontLeftModule.getDriveVelocity());
        // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Front right drive velocity", () -> frontRightModule.getDriveVelocity());
        // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Back left drive velocity", () -> backLeftModule.getDriveVelocity());
        // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Back right drive velocity", () -> backRightModule.getDriveVelocity());

        // // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Front left drive voltage", () -> frontLeftModule.getDriveVoltage());
        // // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Front right drive voltage", () -> frontRightModule.getDriveVoltage());
        // // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Back left drive voltage", () -> backLeftModule.getDriveVoltage());
        // // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Back right drive voltage", () -> backRightModule.getDriveVoltage());

        // // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Front left target angle", () -> states[0].angle.getDegrees());
        // // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Front right target angle", () -> states[1].angle.getDegrees());
        // // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Back left target angle", () -> states[2].angle.getDegrees());
        // // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Back right target angle", () -> states[3].angle.getDegrees());

        // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Front left target velocity", () -> states[0].speedMetersPerSecond);
        // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Front right target velocity", () -> states[1].speedMetersPerSecond);
        // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Back left target velocity", () -> states[2].speedMetersPerSecond);
        // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Back right target velocity", () -> states[3].speedMetersPerSecond);

        // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Pigeon heading", () -> getYaw2d().getDegrees());
        // LightningShuffleboard.setDoubleSupplier("Drivetrain", "Odometry heading", () -> getHeading().getDegrees());
        // LightningShuffleboard.setDoubleSupplier("Drivetrain", "ES X", () -> pose.getX());
        // LightningShuffleboard.setDoubleSupplier("Drivetrain", "ES Y", () -> pose.getY());
        // LightningShuffleboard.setDoubleSupplier("Drivetrain", "ES Z", () -> pose.getRotation().getDegrees());

        // LightningShuffleboard.setDoubleArray("Drivetrain", "ES Pose", () -> new double[]{pose.getX(), pose.getY(), pose.getRotation().getRadians()});

    }

    /**
     * Gets the current pathplanner path point of the robot in meters using
     */
    public PathPoint getCurrentPathPoint() {
        return new PathPoint(pose.getTranslation(), pose.getRotation());
    }

    // /**
    //  * Sets initial pose of robot in meters.
    //  * 
    //  * @param initalPosition the initial position of the robot
    //  * @param initalRotation the initial rotation(heading) of the robot
    //  */
    // public void setInitialPose(Pose2d initalPosition, Rotation2d initalRotation) {
    //     pigeon.setYaw(initalRotation.getDegrees());
    //     pose = new Pose2d(initalPosition.getTranslation(), initalRotation);
    //     estimator = new SwerveDrivePoseEstimator(kinematics, getYaw2d(), modulePositions, pose);

    // }

    /**
     * Gets the heading of the robot from odometry in degrees from 0 to 360
     */
    public Rotation2d getHeading() {
        return pose.getRotation();
    }

    /**
     * Gets the current rotation from the pigeon
     * 
     * @return the current heading of the robot in degrees from 0 to 360
     */
    public Rotation2d getYaw2d() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(pigeon.getYaw() - 90, 0, 360));
    }

    /**
     * Gets the current pitch of the robot from the pigeon
     * 
     * @return the current pitch of the robot in degrees from -180 to 180
     */
    public Rotation2d getPitch2d() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(pigeon.getPitch(), -180, 180));
    }

    /**
     * Gets the current roll of the robot from the pigeon
     * 
     * @return the current roll of the robot in degrees from -180 to 180
     */
    public Rotation2d getRoll2d() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(pigeon.getRoll(), -180, 180));
    }

    /**
     * Gets current state of module.
     * 
     * @return the current state of the specified module
     */
    public SwerveModuleState stateFromModule(SwerveModule swerveModule) {
        return new SwerveModuleState(swerveModule.getDriveVelocity(), new Rotation2d(swerveModule.getSteerAngle()));
    }

    /**
     * Converts percent output of joystick to a velocity in meters per second.
     * 
     * @param percentOutput the percent output of the joystick
     * 
     * @return the velocity in meters per second
     */
    public double percentOutputToMetersPerSecond(double percentOutput) {
        return percentOutput * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
    }

    /**
     * Converts percent output of joystick to a rotational velocity in omega radians per second.
     * 
     * @param percentOutput the percent output of the joystick
     * 
     * @return
     */
    public double percentOutputToRadiansPerSecond(double percentOutput) {
        return percentOutput * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    /**
     * Zeroes the yaw of the pigeon.
     */
    public void zeroHeading() {
        pigeon.reset();
        // pigeon.setYaw(0);
    }
    // public void setHeading(double input) {
    //     pigeon.setYaw(input);
    //     // pigeon.setAngleAdjustment(input);
    // }

    /**
     * Gets the current pose of the robot.
     */
    public Pose2d getPose() {
        return pose;
    }

    /**
     * Resets the odometry to the specified pose.
     * 
     * @param pose the pose to which to set the odometry
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getYaw2d(), modulePositions, pose);
    }

    /**
     * Gets the kinematics of the robot.
     * 
     * @return the kinematics of the robot
     */
    public SwerveDriveKinematics getDriveKinematics() {
        return kinematics;
    }

    /**
     * Gets the states of the modules.
     * 
     * @return the states of the modules
     */
    public SwerveModuleState[] getStates() {
        return states;
    }

    /**
     * Gets the front left module.
     * 
     * @return the front left module
     */
    public SwerveModule getFrontLeftModule() {
        return frontLeftModule;
    }

    /**
     * Gets the front right module.
     * 
     * @return the front right module
     */
    public SwerveModule getFrontRightModule() {
        return frontRightModule;
    }

    /**
     * Gets the back left module.
     * 
     * @return the back left module
     */
    public SwerveModule getBackLeftModule() {
        return backLeftModule;
    }

    /**
     * Gets the back right module.
     * 
     * @return the back right module
     */
    public SwerveModule getBackRightModule() {
        return backRightModule;
    }

    /**
     * Gets the chassis speeds.
     * 
     * @return the chassis speeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }

    /**
     * Sets the chassis speeds.
     * 
     * @param chassisSpeeds the chassis speeds to set
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    /**
     * Sets all motor speeds to 0 and sets the modules to their respective resting angles
     */
    public void stop() {
        frontLeftModule.set(0, DrivetrainConstants.FRONT_LEFT_RESTING_ANGLE);
        frontRightModule.set(0, DrivetrainConstants.FRONT_RIGHT_RESTING_ANGLE);
        backLeftModule.set(0, DrivetrainConstants.BACK_LEFT_RESTING_ANGLE);
        backRightModule.set(0, DrivetrainConstants.BACK_RIGHT_RESTING_ANGLE);

    }

    public void resetNeoAngle() {
        frontLeftModule.setEncoderAngle();
        frontRightModule.setEncoderAngle();
        backLeftModule.setEncoderAngle();
        backRightModule.setEncoderAngle();
    }
}

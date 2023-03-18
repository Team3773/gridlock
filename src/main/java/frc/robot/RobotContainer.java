package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.HashMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ServoTurn;
// import frc.robot.subsystems.ShuffleBoard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.tests.DriveTrainSystemTest;
import frc.robot.subsystems.Drivetrain;
import frc.thunder.LightningContainer;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.LimelightConstants;
import frc.thunder.auto.AutonomousCommandFactory;
import frc.thunder.filter.JoystickFilter;
import frc.thunder.filter.JoystickFilter.Mode;
import frc.thunder.pathplanner.com.pathplanner.lib.PathConstraints;
import frc.thunder.testing.SystemTest;

public class RobotContainer extends LightningContainer {

    private static final LimelightFront frontLimelight = new LimelightFront(LimelightConstants.FRONT_NAME, LimelightConstants.FRONT_POSE);
    private static final LimelightBack backLimelight = new LimelightBack(LimelightConstants.BACK_NAME, LimelightConstants.BACK_POSE);

    // Creating our main subsystems
    private static final Drivetrain drivetrain = new Drivetrain();
    // private static final ShuffleBoard shuffleboard = new ShuffleBoard(drivetrain, elevator, arm, wrist, collector);

    // Creates our controllers and deadzones
    private static final XboxController driver = new XboxController(XboxControllerConstants.DRIVER_CONTROLLER_PORT);
    private static final XboxController copilot = new XboxController(XboxControllerConstants.COPILOT_CONTROLLER_PORT);
    private static final JoystickFilter joystickFilter = new JoystickFilter(XboxControllerConstants.DEADBAND, XboxControllerConstants.MIN_POWER, XboxControllerConstants.MAX_POWER, Mode.CUBED);

    // creates Autonomous Command
    private static final AutonomousCommandFactory autoFactory = new AutonomousCommandFactory(drivetrain::getPose, drivetrain::resetOdometry, drivetrain.getDriveKinematics(),
            AutonomousConstants.DRIVE_PID_CONSTANTS, AutonomousConstants.THETA_PID_CONSTANTS, AutonomousConstants.POSE_PID_CONSTANTS, drivetrain::setStates, drivetrain::resetNeoAngle, drivetrain);

    @Override
    protected void configureButtonBindings() {
        /* driver Controls */
        // RESETS
        new Trigger(driver::getBackButton).onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));
        // new Trigger(driver::getStartButton).onTrue(new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d())));
        // new Trigger(driver::getStartButton).onTrue(new InstantCommand(() -> drivetrain.setHeading(180)));

        new Trigger(driver::getAButton).onTrue(new InstantCommand(drivetrain::resetNeoAngle));

        // GAME PIECE SET


        //SET DRIVE PODS TO 45
        new Trigger(driver::getXButton).whileTrue(new RunCommand(() -> drivetrain.stop(), drivetrain));

        //AUTO ALIGN
        new Trigger(driver::getYButton).whileTrue(new AutoAlign(drivetrain, frontLimelight));

        // new Trigger(driver::getBButton).onTrue(new InstantCommand(() -> servoturn.turnServo(AutonomousConstants.SERVO_DEPLOY)));
        // new Trigger(driver::getBButton).onFalse(new InstantCommand(() -> servoturn.turnServo(AutonomousConstants.SERVO_STOW)));


        //AUTOBALANCE
        // new Trigger(driver::getBButton).whileTrue(new AutoBalance(drivetrain));

        /* copilot controls */
        //BIAS

        //SETPOINTS

    }

    // Creates the autonomous commands
    @Override
    protected void configureAutonomousCommands() {
        //Test paths 

        // // Game paths
        // //A paths
        // // autoFactory.makeTrajectory("A1[2]-M", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
        // //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // // autoFactory.makeTrajectory("A1[2]-M-C", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
        // //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // // autoFactory.makeTrajectory("A1[3]-M", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
        // //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // autoFactory.makeTrajectory("A2[1]-M", Maps.getPathMap(drivetrain, servoturn, lift, collector),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // autoFactory.makeTrajectory("A2[1]-M-HIGH", Maps.getPathMap(drivetrain, servoturn, lift, collector), 
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // autoFactory.makeTrajectory("A2[1]-M-C", Maps.getPathMap(drivetrain, servoturn, lift, collector),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // //B paths
        // autoFactory.makeTrajectory("B2[1]-C", Maps.getPathMap(drivetrain, servoturn, lift, collector),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // // autoFactory.makeTrajectory("B2[1]-M-C", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
        // //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // // autoFactory.makeTrajectory("B2[2]-M-C", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
        // //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // autoFactory.makeTrajectory("B2[1]-C-HIGH", Maps.getPathMap(drivetrain, servoturn, lift, collector),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // //C paths
        // autoFactory.makeTrajectory("C2[1]-M", Maps.getPathMap(drivetrain, servoturn, lift, collector),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // autoFactory.makeTrajectory("C2[1]-M-HIGH", Maps.getPathMap(drivetrain, servoturn, lift, collector), 
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // autoFactory.makeTrajectory("C2[1]-M-C", Maps.getPathMap(drivetrain, servoturn, lift, collector),
        //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // // autoFactory.makeTrajectory("C2[2]-M", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
        // //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // // autoFactory.makeTrajectory("C2[2]-M-C", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
        // //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        // // autoFactory.makeTrajectory("C2[3]-M", Maps.getPathMap(drivetrain, servoturn, lift, collector, leds),
        // //         new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
    }

    @Override
    protected void configureDefaultCommands() {
        /*
         * Set up the default command for the drivetrain. The controls are for field-oriented driving: Left
         * stick Y axis -> forward and backwards movement Left stick X axis -> left and right movement Right
         * stick X axis -> rotation
         */
        // drivetrain.setDefaultCommand(new SwerveDrive(drivetrain, () -> -joystickFilter.filter(driver.getLeftX() * Math.sqrt(2)), () -> joystickFilter.filter(driver.getLeftY() * Math.sqrt(2)),
        //         () -> -joystickFilter.filter(driver.getRightX())));
        drivetrain.setDefaultCommand(new SwerveDrive(drivetrain, () -> MathUtil.applyDeadband(driver.getLeftX(), XboxControllerConstants.DEADBAND),
                () -> MathUtil.applyDeadband(driver.getLeftY(), XboxControllerConstants.DEADBAND), () -> MathUtil.applyDeadband(-driver.getRightX(), XboxControllerConstants.DEADBAND),
                () -> driver.getRightTriggerAxis() > 0.25));

        // elevator.setDefaultCommand(
        // new ManualLift(() -> driver.getRightTriggerAxis() - driver.getLeftTriggerAxis(),
        // () -> 0, () -> 0, arm, wrist, elevator));
        // collector.setDefaultCommand(new Collect(collector, () -> MathUtil.applyDeadband(copilot.getRightTriggerAxis(), XboxControllerConstants.DEADBAND)
        //         - MathUtil.applyDeadband(copilot.getLeftTriggerAxis(), XboxControllerConstants.DEADBAND)));
    }

    @Override
    protected void configureSystemTests() {
        SystemTest.registerTest("fl drive test", new DriveTrainSystemTest(drivetrain, drivetrain.getFrontLeftModule(), 0.25));

        SystemTest.registerTest("fr drive test", new DriveTrainSystemTest(drivetrain, drivetrain.getFrontRightModule(), 0.25));

        SystemTest.registerTest("bl drive test", new DriveTrainSystemTest(drivetrain, drivetrain.getBackLeftModule(), 0.25));

        SystemTest.registerTest("br drive test", new DriveTrainSystemTest(drivetrain, drivetrain.getBackRightModule(), 0.25));
    }

    @Override
    protected void releaseDefaultCommands() {}

    @Override
    protected void initializeDashboardCommands() {}

    @Override
    protected void configureFaultCodes() {}

    @Override
    protected void configureFaultMonitors() {}

    @Override
    protected AutonomousCommandFactory getCommandFactory() {
        return autoFactory;
    }
}

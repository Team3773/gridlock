package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFront;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class AutoAlign extends CommandBase {
    private Drivetrain drivetrain;
    private LimelightFront limelight;
    private PIDController controller;

    private double OFFSET;

    public AutoAlign(Drivetrain drivetrain, LimelightFront limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;

        if (Constants.isBlackout()) {
            // If blackout, use the blackout offset
            OFFSET = AutoAlignConstants.LIMELGHT_OFFSET_BLACKOUT;
        } else {
            // Otherwise, assume gridlock offset
            OFFSET = AutoAlignConstants.LIMELGHT_OFFSET_GRIDLOCK;
        }
        
        controller = new PIDController(AutoAlignConstants.AUTO_ALIGN_PID_CONSTANTS.kP, AutoAlignConstants.AUTO_ALIGN_PID_CONSTANTS.kI, AutoAlignConstants.AUTO_ALIGN_PID_CONSTANTS.kD);
        controller.setSetpoint(OFFSET);
        controller.setTolerance(AutoAlignConstants.TOLERANCE);
        
        addRequirements(drivetrain, limelight);
    }

    @Override
    public void initialize() {
        limelight.setPipelineNum(2);
    }

    @Override
    public void execute() {
        LightningShuffleboard.setBool("Auto align", "OnTarget", onTarget());
        LightningShuffleboard.setDouble("Auto align", "Horizontal offset", limelight.getHorizontalOffset() - OFFSET);

        if (limelight.hasVision()) {
            drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                    drivetrain.percentOutputToMetersPerSecond(
                            controller.calculate(limelight.getHorizontalOffset())),
                    drivetrain.percentOutputToMetersPerSecond(0d),
                    drivetrain.percentOutputToRadiansPerSecond(0d),
                    drivetrain.getYaw2d()));
        } else {
            drivetrain.stop();
        }
    }

    public boolean onTarget() {
        double currentAngle = limelight.getHorizontalOffset();
        currentAngle -= OFFSET;
        return Math.abs(currentAngle) < AutoAlignConstants.TOLERANCE;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        limelight.setPipelineNum(0);
        drivetrain.stop();
    }
}

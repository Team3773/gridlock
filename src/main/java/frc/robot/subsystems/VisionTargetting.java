package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.thunder.limelightlib.LimelightHelpers;
import frc.thunder.logging.DataLogger;
import frc.thunder.shuffleboard.LightningShuffleboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class VisionTargetting extends SubsystemBase {

    // Change "limelight-alice" to whatever the name of the limelight you are
    // currently using Alice
    // we should rename the limelight names to something consistent later
    private String limelightName = "limelight-alice";


    // Setting values that we want to get later
    private double horizAngleToTarget;

    // Fiducial values
    private double[] botPose = LimelightHelpers.getBotPose(limelightName);
    private double[] botPoseBlue = LimelightHelpers.getBotPose_wpiBlue(limelightName);
    private double[] botPoseRed = LimelightHelpers.getBotPose_wpiRed(limelightName);

    // Both Fiducial and RetroReflective
    private boolean hasVision = LimelightHelpers.getTV(limelightName);

    // Pipeline 0 is Fiducial (april tags), Pipeline 1 is RetroReflective (tape)
    private int pipelineNum = 0;

    // RetroReflective values
    private double horizontalOffset = LimelightHelpers.getTX(limelightName);
    private double verticalOffset = LimelightHelpers.getTY(limelightName);
    private double targetVertical = LimelightHelpers.getTA(limelightName);


    public VisionTargetting() {
        // Inits logging for vision
        initLogging();

        // Registers this as a proper Subsystem
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {


        // Updates all of our values used in VisionTargeting
        botPose = LimelightHelpers.getBotPose(limelightName);
        botPoseBlue = LimelightHelpers.getBotPose_wpiBlue(limelightName);
        botPoseRed = LimelightHelpers.getBotPose_wpiRed(limelightName);
        hasVision = LimelightHelpers.getTV(limelightName);
        horizontalOffset = LimelightHelpers.getTX(limelightName);
        verticalOffset = LimelightHelpers.getTY(limelightName);
        targetVertical = LimelightHelpers.getTA(limelightName);

        // Sets the pipeline to the current one set on shuffleboard
        setPipeline();

        if (hasVision) {
            // Updating the Shuffleboard if we have vision

            // Fiducial (april tag) Values
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose TX", botPose[0]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose TY", botPose[1]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose RZ", botPose[5]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose Blue TX", botPoseBlue[0]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose Blue TY", botPoseBlue[1]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose Blue RZ", botPoseBlue[5]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose Red TX", botPoseRed[0]);

            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose Red TY", botPoseRed[1]);
            LightningShuffleboard.setDouble("Autonomous", "1Vision bot pose Red RZ", botPoseRed[5]);

            // RetroReflective Values
            LightningShuffleboard.setDouble("Autonomous", "1RR Tape Horizontal Offset", horizontalOffset);
            LightningShuffleboard.setDouble("Autonomous", "1RR Tape Vertical Offset", verticalOffset);
            LightningShuffleboard.setDouble("Autonomous", "1RR Tape Target Area", targetVertical);
        }

    }


    // Adds logging for vision so we can look at values when the robot is off and check them
    public void initLogging() {
        if (hasVision) {
            DataLogger.addDataElement("Vision bot pose TX", () -> botPose[0]);
            DataLogger.addDataElement("Vision bot pose TY", () -> botPose[1]);
            DataLogger.addDataElement("Vision bot pose RZ", () -> botPose[5]);
            DataLogger.addDataElement("Vision bot pose Blue TX", () -> botPoseBlue[0]);
            DataLogger.addDataElement("Vision bot pose Blue TY", () -> botPoseBlue[1]);
            DataLogger.addDataElement("Vision bot pose Blue RZ", () -> botPoseBlue[5]);
            DataLogger.addDataElement("Vision bot pose Red TX", () -> botPoseRed[0]);
            DataLogger.addDataElement("Vision bot pose Red TY", () -> botPoseRed[1]);
            DataLogger.addDataElement("Vision bot pose Red RZ", () -> botPoseRed[5]);
            DataLogger.addDataElement("Vision retro reflective TX", () -> horizontalOffset);
            DataLogger.addDataElement("Vision retro reflective TY", () -> verticalOffset);
            DataLogger.addDataElement("Vision retro reflective TA", () -> targetVertical);
        }
    }

    // Returns the robot pose as a Pose2d from vision data
    public Pose2d getRobotPose() {
        if (hasVision) {
            return new Pose2d(new Translation2d(botPoseBlue[0], botPoseBlue[1]),
                    Rotation2d.fromDegrees(botPoseBlue[5]));
        } else {
            return null;
        }
    }

    /**
     * Gets botpose on field (x,y,z,rx,ry,rz)
     * 
     * @return 3d bot pose
     */
    public double[] getBotPose() {
        return this.botPose;
    }

    /**
     * Sets the pipeline we're using on the limelight. The first is for april tag
     * targetting The
     * second is for retroreflective tape.
     * 
     * @param pipelineNum The pipeline number being used on the limelight.
     */
    public void setPipelineNum(int pipelineNum) {
        LimelightHelpers.setPipelineIndex(limelightName, pipelineNum);
    }

    public double getPipelineNum() {
        return this.pipelineNum;
    }

    /**
     * Ensures that what we're receiving is actually a valid target (if it's outside
     * of FOV, it
     * can't be)
     * 
     * @return Whether or not target offset is more than 29.8 degrees.
     */
    public boolean validTarget() {
        // 29.8d represents the LL2+'s max FOV, from center of camera to edge of frame.
        return Math.abs(this.horizAngleToTarget) < Constants.Vision.HORIZ_CAMERA_FOV;
    }

    /**
     * Gives us degree offset to adjust our rotation by.
     * 
     * @return degree offset from target.
     */
    public double autoAlign() {
        // Set pipeline num to 2, should be retroreflective tape pipeline.
        setPipelineNum(2);

        this.horizAngleToTarget = LimelightHelpers.getTX(limelightName);

        boolean isOnTarget = isOnTarget(this.horizAngleToTarget);

        // Checks our current angle on the target
        if (hasVision && !isOnTarget && validTarget()) {
            return horizAngleToTarget;
        } else {
            return 0d;
        }
    }

    /**
     * Function to tell us whether or not we're on target (centered on vision tape)
     * 
     * @param expectedAngle Angle we're supposed to be at according to offset of
     *                      target supplied by
     *                      Limelight
     * @return Whether we're within acceptable tolerance of the target.
     */
    public boolean isOnTarget(double expectedAngle) {
        // Should put consideration into how accurate we want to be later on.
        return expectedAngle < Constants.Vision.HORIZ_DEGREE_TOLERANCE;
    }

    private void setPipeline() {

        // Gets the current shuffleboard value for the Pipeline entry
        pipelineNum = (int) LimelightHelpers.getLimelightNTDouble("limelight", "pipeline");

        // Updates the Limelight pipeline accordingly if pipelineNum is different than the current pipeline
        if (pipelineNum != getPipelineNum()){
            LimelightHelpers.setLimelightNTDouble("limelight", "pipeline", pipelineNum);
        }
    }
}

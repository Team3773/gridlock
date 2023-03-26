package frc.robot.commands;

import frc.robot.Constants.OperationConstants;
import frc.robot.subsystems.ArmExtendSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmExtendPIDCommand extends CommandBase{
    private final ArmExtendSubsystem armExtendSubsystem;
    private final PIDController pidController;
    private double setpoint;
    private int extendCounter = 0;

    public ArmExtendPIDCommand(ArmExtendSubsystem subsystem, double setpoint)
    {
        this.setpoint = setpoint;
        armExtendSubsystem = subsystem;
        this.pidController = new PIDController(0.15, 0, 0); 
        pidController.setSetpoint(setpoint);
        
        addRequirements(armExtendSubsystem);
    }
    @Override
    public void initialize() {
        pidController.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = pidController.calculate(armExtendSubsystem.getEncoderMeters());
        armExtendSubsystem.setArmExtendSpeed(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        armExtendSubsystem.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if((armExtendSubsystem.getEncoderMeters() < (setpoint + OperationConstants.setpointDeadband)) && (armExtendSubsystem.getEncoderMeters() > (setpoint - OperationConstants.setpointDeadband))){
            extendCounter += 1;
          }else{
            extendCounter = 0;
          }
          if(extendCounter >= 20){
            return true;
          }
          return false;
    }
}
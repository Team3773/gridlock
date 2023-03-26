package frc.robot.commands;

import frc.robot.Constants.OperationConstants;
import frc.robot.subsystems.ArmRotateSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmRotatePIDCommand extends CommandBase{
    private final ArmRotateSubsystem armRotateSub;
    private final PIDController pidController;
    private double setpoint;
    private int armRotateCounter = 0;

    public ArmRotatePIDCommand(ArmRotateSubsystem subsystem, double setpoint)
    {
        this.setpoint = setpoint;
        armRotateSub = subsystem;
        this.pidController = new PIDController(0.5, 0, 0); 
        pidController.setSetpoint(setpoint);
        
        addRequirements(armRotateSub);
    }
    @Override
    public void initialize() {
        pidController.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = pidController.calculate(armRotateSub.getEncoderMeters());
        armRotateSub.setArmRotateSpeed(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        armRotateSub.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if((armRotateSub.getEncoderMeters() < (setpoint + OperationConstants.setpointDeadband)) && (armRotateSub.getEncoderMeters() > (setpoint - OperationConstants.setpointDeadband))){
            armRotateCounter += 1;
          }else{
            armRotateCounter = 0;
          }
          if(armRotateCounter >= 20){
            return true;
          }
          return false;
    }
}
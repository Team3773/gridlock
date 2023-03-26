package frc.robot.commands;

import frc.robot.Constants.OperationConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ElevatorPIDCommand extends CommandBase{
    private final ElevatorSubsystem elevatorSub;
    private final PIDController pidController;
    private double setpoint;
    private int elevatorCounter = 0;


    public ElevatorPIDCommand(ElevatorSubsystem subsystem, double setpoint)
    {
        this.setpoint = setpoint;
        elevatorSub = subsystem;
        this.pidController = new PIDController(0.15, 0, 0); 
        pidController.setSetpoint(setpoint);
        
        addRequirements(elevatorSub);
    }
    @Override
    public void initialize() {
        pidController.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = pidController.calculate(elevatorSub.getEncoderMeters());
        elevatorSub.setElevatorSpeed(speed);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevatorSub.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if((elevatorSub.getEncoderMeters() < (setpoint + OperationConstants.setpointDeadband)) && (elevatorSub.getEncoderMeters() > (setpoint - OperationConstants.setpointDeadband))){
            elevatorCounter += 1;
          }else{
            elevatorCounter = 0;
          }
          if(elevatorCounter >= 20){
            return true;
          }
          return false;
    }    
}
package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ElevatorCommand extends CommandBase{
    private final ElevatorSubsystem elevatorSubsystem;
    private double speed;

    public ElevatorCommand(ElevatorSubsystem subsystem, double speed)
    {
        elevatorSubsystem = subsystem;
        this.speed = speed;
        
        addRequirements(elevatorSubsystem);
    }
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevatorSubsystem.setElevatorSpeed(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
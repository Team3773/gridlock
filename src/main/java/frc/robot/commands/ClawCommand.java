package frc.robot.commands;

import frc.robot.subsystems.ClawSubsystem;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawCommand extends CommandBase{
    private final ClawSubsystem clawSub;
    private Supplier<Double> openSpeedFunction;
    private Supplier<Double> closeSpeedFunction;

    public ClawCommand(ClawSubsystem subsystem, Supplier<Double> closeSpeedFunction, Supplier<Double> openSpeedFunction)
    {
        clawSub = subsystem; 
        this.closeSpeedFunction = closeSpeedFunction;
        this.openSpeedFunction = openSpeedFunction;
        
        addRequirements(clawSub);
    }
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double clawSpeed = 0;
        double openSpeed = openSpeedFunction.get();
        double closeSpeed = closeSpeedFunction.get();

        SmartDashboard.putNumber("Open Speed", openSpeed);
        SmartDashboard.putNumber("Close Speed", closeSpeed);

        // If close speed is almost zero and open speed is abs greater than zero, open
        // Deadband of 0.05
        if(Math.abs(closeSpeed) < 0.05 && Math.abs(openSpeed) > 0.05)
        {
            clawSpeed = openSpeed;
        }
        else if(Math.abs(closeSpeed) > 0.05 && Math.abs(openSpeed) < 0.05)
        {
            clawSpeed = -closeSpeed;
        }
        else
        {
            clawSpeed = 0;
        }

        clawSub.setClawSpeed(clawSpeed * 0.25);
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
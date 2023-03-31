package frc.robot.commands;

import frc.robot.subsystems.ArmExtendSubsystem;
import frc.robot.subsystems.ArmRotateSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmRotateCommand extends CommandBase{
    private final ArmRotateSubsystem armRotateSub;
    private final ArmExtendSubsystem armExtendSub;
    private final Supplier<Double> rotateSpeedFunction;
    private final BooleanSupplier ignoreZero;

    public ArmRotateCommand(ArmRotateSubsystem subsystem, ArmExtendSubsystem armExtendSub, Supplier<Double> rotateSpeedFunction, BooleanSupplier ignoreZero)
    {
        this.ignoreZero = ignoreZero;
        armRotateSub = subsystem;
        this.armExtendSub = armExtendSub;
        this.rotateSpeedFunction = rotateSpeedFunction; 
        
        addRequirements(armRotateSub);
    }
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        boolean ignore = ignoreZero.getAsBoolean();
        double rotateSpeed = rotateSpeedFunction.get();
        // rotateSpeed *= -1;


        if(armExtendSub.isExtendAtZero() && armRotateSub.isRotateAtZero() && !ignore)
        {
            if(rotateSpeed < 0)
            {
                armExtendSub.stopMotor();
                armRotateSub.stopMotor();
            }
            else
            {
                armRotateSub.setArmRotateSpeed(rotateSpeed);
            }
        }
        else
        {
            // if(Math.abs(rotateSpeed) < 0.12)
            // {
            //         rotateSpeed = 0;
            // }
            
            armRotateSub.setArmRotateSpeed(rotateSpeed);
        }
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
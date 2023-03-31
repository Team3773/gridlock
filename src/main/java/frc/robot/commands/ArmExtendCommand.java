package frc.robot.commands;

import frc.robot.subsystems.ArmExtendSubsystem;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmRotateSubsystem;

public class ArmExtendCommand extends CommandBase{
    private final ArmExtendSubsystem armExtendSub;
    private final ArmRotateSubsystem armRotateSub;
    private final Supplier<Double> extendSpeedFunction;

    public ArmExtendCommand(ArmExtendSubsystem subsystem, ArmRotateSubsystem armRotateSub, Supplier<Double> extendSpeedFunction)
    {
        armExtendSub = subsystem;
        this.extendSpeedFunction = extendSpeedFunction; 
        this.armRotateSub = armRotateSub;
        
        addRequirements(armExtendSub);
    }

    // Runs when command starts
    @Override
    public void initialize() {
    }

    // Runs repeatedly when command is called
    @Override
    public void execute() {
        double extendSpeed = extendSpeedFunction.get();

        if(armExtendSub.isExtendAtZero() && armRotateSub.isRotateAtZero())
        {
            if(extendSpeed < 0)
            {
                armExtendSub.stopMotor();
                armRotateSub.stopMotor();
            }
            else
            {
                armExtendSub.setArmExtendSpeed(extendSpeed);
            }
        }
        else
        {   
            armExtendSub.setArmExtendSpeed(extendSpeed);
        }

        // DEADBAND
        // if(Math.abs(extendSpeed) < 0.12)
        // {
        //     extendSpeed = 0;
        // }

        // armExtendSub.setArmExtendSpeed(extendSpeed);
    }

    // Runs when command ends
    @Override
    public void end(boolean interrupted) {
    }

    // Returns when command is finished. Can also be interupted in container.
    @Override
    public boolean isFinished() {
        return false;
    }
}
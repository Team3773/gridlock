package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.LiftConstants.LiftState;
import frc.robot.subsystems.Lift;

public class Stow extends InstantCommand {
    Lift lift;
    public Stow(Lift lift) {
        this.lift = lift;

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        lift.setGoalState(LiftState.stowed);
    }
}

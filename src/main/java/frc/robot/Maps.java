package frc.robot;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.commands.ArmExtendPIDCommand;
import frc.robot.commands.ArmRotateCommand;
import frc.robot.commands.ArmRotatePIDCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ServoTurn;
import frc.robot.subsystems.ArmExtendSubsystem;
import frc.robot.subsystems.ArmRotateSubsystem;

/**
 * Class for creating all auton HasMaps
 */
public class Maps {
    /**
     * The general Hash map for all paths. 
     * Has most calls needed for the paths to run.
     * NEED TO FIX score piece 
     * 
     * @param drivetrain
     * @return
     */
    public static HashMap<String, Command> getPathMap(Drivetrain drivetrain, ArmExtendSubsystem armExtendSubsystem) {

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Fling", new ArmExtendPIDCommand(armExtendSubsystem, 9));
        // eventMap.put("Fling", new RunCommand(() -> armRotateSub.setArmRotateSpeed(0.3).until(armRotateSub.stopArm())));

        // new ArmRotatePIDCommand(armRotateSub, 30));
        // eventMap.put("Fling", new InstantCommand(() -> armRotateSub.setArmRotateSpeed(.5)));
        // eventMap.put("Ground-Collect-Cone", new RunCommand(() -> lift.setGoalState(LiftState.groundCone), lift).until(lift::goalReached));
        // eventMap.put("Ground-Collect-Cube", new RunCommand(() -> lift.setGoalState(LiftState.groundCube), lift).until(lift::goalReached));
        // eventMap.put("Ground-Score", new RunCommand(() -> lift.setGoalState(LiftState.groundCube), lift).until(lift::goalReached));  
        // eventMap.put("High-Score-Cone", new RunCommand(() -> lift.setGoalState(LiftState.highConeScore), lift).until(lift::goalReached));
        // eventMap.put("High-Score-Cube", new RunCommand(() -> lift.setGoalState(LiftState.highCubeScore), lift).until(lift::goalReached));
        // eventMap.put("Stow", new RunCommand(() -> lift.setGoalState(LiftState.stowed), lift).until(lift::goalReached));   
        // eventMap.put("Stop-Collect", new InstantCommand(() -> collector.stop(), collector));
        // eventMap.put("Hold-Power", new InstantCommand(() -> collector.setPower(CollectorConstants.HOLD_POWER), collector));
        // eventMap.put("Collect-Piece", new Collect(collector, () -> .5d));
        // eventMap.put("Score-Piece", new Collect(collector, () -> -.5d)); //TODO: switch until to be until no piece
        // eventMap.put("Auto-Balance", new AutoBalance(drivetrain));
        // eventMap.put("Update-Pos-Vision", new PrintCommand("Update-Pos-Vision")); //TODO add vision code
        return eventMap;
    }
}

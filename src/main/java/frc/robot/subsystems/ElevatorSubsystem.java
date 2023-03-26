package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.OperationConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    CANSparkMax elevatorMotor = new CANSparkMax(OperationConstants.elevatorMotorChannel, MotorType.kBrushless);
    RelativeEncoder elvatorEncoder = elevatorMotor.getEncoder();

    public ElevatorSubsystem() 
    {
      // RESET IN START POSITION
      elvatorEncoder.setPosition(0);
    }
    
      @Override
      public void periodic() {
        elvatorEncoder.setPositionConversionFactor(OperationConstants.kElevatorEncoderRot2Meter);
        SmartDashboard.putNumber("Elevator Encoder: ", getEncoderMeters());
        // This method will be called once per scheduler run
      }
    
      @Override
      public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
      }

      public void setElevatorSpeed(double speed)
      {
        elevatorMotor.set(speed);
      }

      public void stopMotor()
      {
        elevatorMotor.set(0);
      }

      public void zeroEncoder()
      {
        elvatorEncoder.setPosition(0);
      }

      public double getEncoderMeters() {
        return elvatorEncoder.getPosition();
      }
}

        // CODE TO SOP ELEVATOR FROM MAXING OR MINING
        // if(elvatorEncoder.getPosition() >= OperationConstants.kElevatorTopPosition)
        // {
        //   if (speed < 0)
        //   {
        //     elevatorMotor.set(speed);
        //   }
        // }
        // else if(elvatorEncoder.getPosition() <= OperationConstants.kElevatorBottomPosition)
        // {
        //   if (speed > 0)
        //   {
        //     elevatorMotor.set(speed);
        //   }
        // }
        // else
        // {
        //   elevatorMotor.set(speed);
        // }
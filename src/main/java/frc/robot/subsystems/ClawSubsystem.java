package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.OperationConstants;

public class ClawSubsystem extends SubsystemBase{
    
    public ClawSubsystem() 
    {
      // RESET IN START POSITION
      clawEncoder.setPosition(0);
      clawMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    }
      CANSparkMax clawMotor = new CANSparkMax(OperationConstants.clawMotorChannel, MotorType.kBrushless);
      RelativeEncoder clawEncoder = clawMotor.getEncoder();

      @Override
      public void periodic() {
        clawEncoder.setPositionConversionFactor(OperationConstants.kClawEncoderRot2Meter);
        SmartDashboard.putNumber("Claw Encoder: ", getEncoderMeters());
        // This method will be called once per scheduler run
      }
    
      @Override
      public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
      }
      public void setClawSpeed(double speed)
      {
        clawMotor.set(speed);
      }

      public void zeroEncoder()
      {
        clawEncoder.setPosition(0);
      }

      public void stopMotor()
      {
        clawMotor.set(0);
      }

    public double getEncoderMeters() {
      return clawEncoder.getPosition();
    }
}
        // CODE TO STOP CLAW MOTOR FROM MAXING OR MINING
        // if(getEncoderMeters() >= OperationConstants.kClawStopGrabPoint)
        // {
        //   if(speed < 0)
        //   {
        //     clawMotor.set(speed);
        //   }
        // }
        // else
        // {
        //   clawMotor.set(speed);
        // }
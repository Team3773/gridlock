package frc.robot.subsystems;

import frc.robot.Constants.OperationConstants;
import frc.thunder.shuffleboard.LightningShuffleboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmExtendSubsystem extends SubsystemBase{
    
    public ArmExtendSubsystem() 
    {
      // RESET IN START POSITION
      armExtendEncoder.setPosition(0);
    //   armExtendMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 5, 10, 0.5));
    }
      CANSparkMax armExtendMotor = new CANSparkMax(OperationConstants.armExtendMotorChannel, MotorType.kBrushless);
      // CANSparkMax armExtendMotor = new CANSparkMax(OperationConstants.armExtendMotorChannel, MotorType.kBrushless);
      RelativeEncoder armExtendEncoder = armExtendMotor.getEncoder();

    //   Encoder armExtendEncoder = new Encoder(OperationConstants.karmExtendEncoderA, OperationConstants.karmExtendEncoderB);

      @Override
      public void periodic() {
        // armExtendEncoder.setPositionConversionFactor(OperationConstants.kElevatorEncoderRot2Meter);        SmartDashboard.putNumber("Arm Rotate Encoder", armRotateEncoder.getDistance());
        armExtendEncoder.setPositionConversionFactor(OperationConstants.kElevatorEncoderRot2Meter);
        SmartDashboard.putNumber("Arm Extend Encoder: ", getEncoderMeters());
        // SmartDashboard.putNumber("Arm Extend Encoder: ", getEncoderMeters());
        // This method will be called once per scheduler run
        LightningShuffleboard.setDouble("Arm Extend Encoder", "Arm Extend Encoder", armExtendEncoder.getPosition());
      }
    
      @Override
      public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulationjnmjjjnjjnnnmjnmjnmm
      }
      public void setArmExtendSpeed(double speed)
      {
        armExtendMotor.set(speed * OperationConstants.kArmExtendDampner);
      }

      public boolean isExtendAtZero()
      {
        if(armExtendEncoder.getPosition() <= 0)
        {
            return true;
        }
        else
        {
            return false;
        }
      }

      public void stopMotor()
      {
        armExtendMotor.set(0);
      }

      public void zeroEncoder()
      {
        armExtendEncoder.setPosition(0);
      }

      public double getEncoderMeters() {
        return armExtendEncoder.getPosition();
      }
}
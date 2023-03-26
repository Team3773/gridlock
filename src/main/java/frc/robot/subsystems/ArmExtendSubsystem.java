package frc.robot.subsystems;

import frc.robot.Constants.OperationConstants;
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
      armExtendEncoder.reset();
      armExtendMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
    }
      TalonSRX armExtendMotor = new TalonSRX(OperationConstants.armExtendMotorChannel);
      // CANSparkMax armExtendMotor = new CANSparkMax(OperationConstants.armExtendMotorChannel, MotorType.kBrushless);
      Encoder armExtendEncoder = new Encoder(OperationConstants.karmExtendEncoderA, OperationConstants.karmExtendEncoderB);

      @Override
      public void periodic() {
        // armExtendEncoder.setPositionConversionFactor(OperationConstants.kElevatorEncoderRot2Meter);        SmartDashboard.putNumber("Arm Rotate Encoder", armRotateEncoder.getDistance());
        SmartDashboard.putNumber("Arm Rotate Encoder", armExtendEncoder.getDistance());

        // SmartDashboard.putNumber("Arm Extend Encoder: ", getEncoderMeters());
        // This method will be called once per scheduler run
      }
    
      @Override
      public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
      }
      public void setArmExtendSpeed(double speed)
      {
        armExtendMotor.set(ControlMode.PercentOutput, speed * OperationConstants.kArmRotateDampner);
      }

      public void stopMotor()
      {
        armExtendMotor.set(ControlMode.PercentOutput,0);
      }

      public void zeroEncoder()
      {
        armExtendEncoder.reset();
      }

      public double getEncoderMeters() {
        return armExtendEncoder.getDistance();
      }
}

        // CODE TO STOP ARM EXTEND FROM MAXING OR MINING
        // if(armExtendEncoder.getPosition() >= OperationConstants.kElevatorTopPosition)
        // {
        //   if (speed < 0)
        //   {
        //     armExtendMotor.set(speed);
        //   }
        // }
        // else if(armExtendEncoder.getPosition() <= OperationConstants.kElevatorBottomPosition)
        // {
        //   if (speed > 0)
        //   {
        //     armExtendMotor.set(speed);
        //   }
        // }
        // else
        // {
        //   armExtendMotor.set(speed);
        // }
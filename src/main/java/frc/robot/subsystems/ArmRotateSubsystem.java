package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.OperationConstants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmRotateSubsystem extends SubsystemBase{
    
    public ArmRotateSubsystem() 
    {
      // RESET IN START POSITION
      armRotateEncoder.reset();
      armRotateMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
    }
      TalonSRX armRotateMotor = new TalonSRX(OperationConstants.armRotateMotorChannel);
      Encoder armRotateEncoder = new Encoder(OperationConstants.karmRotateEncoderA, OperationConstants.karmRotateEncoderB);

      @Override
      public void periodic() {

        // SmartDashboard.putNumber("Arm Rotate Encoder", getEncoderMeters());
        SmartDashboard.putNumber("Arm Rotate Encoder", armRotateEncoder.getDistance());
        // This method will be called once per scheduler run
      }
    
      @Override
      public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
      }
      public void setArmRotateSpeed(double speed)
      {
        armRotateMotor.set(ControlMode.PercentOutput, speed * OperationConstants.kArmRotateDampner);
      }

      public void stopMotor()
      {
        armRotateMotor.set(ControlMode.PercentOutput, 0);
      }
      public void zeroEncoder()
      {
        armRotateEncoder.reset();
      }
      public double getEncoderMeters() {
        return armRotateEncoder.getDistance();
        // return armRotateEncoder.get() * OperationConstants.kArmRotateEncoderRot2Meter;
      }
}

        // CODE TO PREVENT MAXING OR MINING
        // if(this.getEncoderMeters() >= OperationConstants.kTopArmRotatePoint)
        // {
        //   if (speed < 0)
        //   {
        //     armRotateMotor.set(ControlMode.PercentOutput, speed * OperationConstants.kArmRotateDampner);
        //   }
        // }
        // else if(this.getEncoderMeters() <= OperationConstants.kBottomArmRotatePoint)
        // {
        //   if (speed > 0)
        //   {
        //     armRotateMotor.set(ControlMode.PercentOutput, speed * OperationConstants.kArmRotateDampner);
        //   }
        // }
        // else
        // {
        //   armRotateMotor.set(ControlMode.PercentOutput, speed * OperationConstants.kArmRotateDampner);
        // }
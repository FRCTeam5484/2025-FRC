package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class subElevator extends SubsystemBase {
  SparkMax lowerMotor = new SparkMax(Constants.Elevator.LowerMotor, SparkMax.MotorType.kBrushless);
  SparkMax upperMotor = new SparkMax(Constants.Elevator.UpperMotor, SparkMax.MotorType.kBrushless);
  RelativeEncoder lowerEncoder = lowerMotor.getEncoder();
  RelativeEncoder upperEncoder = upperMotor.getEncoder();
  SparkMaxConfig lowerConfig = new SparkMaxConfig();
  SparkMaxConfig upperConfig = new SparkMaxConfig();

  public subElevator() {
    lowerEncoder.setPosition(0);
    upperEncoder.setPosition(0);

    lowerConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    lowerConfig.encoder
      .positionConversionFactor(1000)
      .velocityConversionFactor(1000);
    lowerConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1.0, 0.0, 0.0);
    lowerMotor.configure(lowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    upperConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .follow(Constants.Elevator.LowerMotor);
    upperConfig.encoder
      .positionConversionFactor(1000)
      .velocityConversionFactor(1000);
    upperConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1.0, 0.0, 0.0);
    upperMotor.configure(upperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void teleOp(double speed) {
    lowerMotor.set(speed);
  }

  public void stop() {
    lowerMotor.stopMotor();
  }

  public void resetEncoders() {
    lowerEncoder.setPosition(0);
    upperEncoder.setPosition(0);
  }
}

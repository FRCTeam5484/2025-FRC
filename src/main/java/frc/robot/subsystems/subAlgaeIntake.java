package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class subAlgaeIntake extends SubsystemBase {
  SparkMax intakeMotor = new SparkMax(Constants.Algae.IntakeMotor, SparkMax.MotorType.kBrushless);
  SparkMaxConfig intakeConfig = new SparkMaxConfig();
  public subAlgaeIntake() {
    intakeConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake);
      //.smartCurrentLimit(20);
    intakeConfig.encoder 
      .positionConversionFactor(1000)
      .velocityConversionFactor(1000);
    intakeConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1.0, 0.0, 0.0);
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void teleOp(double speed) {
    intakeMotor.set(speed);
  }
  public void stop() {
    intakeMotor.stopMotor();
  }
  public void autoIntake(){
    intakeMotor.set(0.6);
  } 
  public void autoHold(){
    intakeMotor.set(0.1);
  }
}

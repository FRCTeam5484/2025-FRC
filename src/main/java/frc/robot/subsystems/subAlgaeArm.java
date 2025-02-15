package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class subAlgaeArm extends SubsystemBase {
  SparkMax armMotor = new SparkMax(Constants.Algae.ArmMotor, SparkMax.MotorType.kBrushless);
  SparkMaxConfig armConfig = new SparkMaxConfig();
  public subAlgaeArm() {
    armConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    armConfig.encoder
      .positionConversionFactor(1000)
      .velocityConversionFactor(1000);
    armConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1.0, 0.0, 0.0);
    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void teleOp_Arm(double speed) {
    armMotor.set(speed);
  }
  public void stop() {
    armMotor.stopMotor();
  }
  public void stopArm() {
    armMotor.stopMotor();
  }
}

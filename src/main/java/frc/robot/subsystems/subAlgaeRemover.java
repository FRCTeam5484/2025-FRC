package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class subAlgaeRemover extends SubsystemBase {
  SparkMax removerMotor = new SparkMax(Constants.Algae.RemoverMotor, SparkMax.MotorType.kBrushless);
  RelativeEncoder removerEncoder = removerMotor.getEncoder();
  PIDController removerPID = new PIDController(0.04, 0.0, 0.0);
  SparkMaxConfig removerConfig = new SparkMaxConfig();
  public subAlgaeRemover() {
    removerConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake);
      //.smartCurrentLimit(20);
    removerConfig.encoder 
      .positionConversionFactor(1000)
      .velocityConversionFactor(1000);
    removerMotor.configure(removerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    resetEncoder();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Remover Encoder", removerEncoder.getPosition());
  }

  public void teleOp(double speed) {
    removerMotor.set(speed);
  }
  public void stop() {
    removerMotor.stopMotor();
  }
  public void autoPosition(double position) {
    removerPID.setSetpoint(position);
    removerMotor.set(removerPID.calculate(removerEncoder.getPosition()));
  }
  public void resetEncoder(){
    removerEncoder.setPosition(0);
  }
}

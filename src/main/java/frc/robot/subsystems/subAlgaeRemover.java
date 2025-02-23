package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class subAlgaeRemover extends SubsystemBase {
  SparkMax removerMotor = new SparkMax(Constants.Algae.RemoverMotor, SparkMax.MotorType.kBrushless);
  RelativeEncoder removerEncoder = removerMotor.getEncoder();
  PIDController removerPID = new PIDController(0.0001, 0.0, 0.0);
  SparkMaxConfig removerConfig = new SparkMaxConfig();
  public double pidSetPoint;
  public subAlgaeRemover() {
    removerConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake);
      //.smartCurrentLimit(20);
    removerConfig.encoder 
      .positionConversionFactor(1000)
      .velocityConversionFactor(1000);
    removerMotor.configure(removerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    resetEncoder();
    pidSetPoint = Constants.Algae.RemoverArmUp;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Remover Encoder", removerEncoder.getPosition());
    SmartDashboard.putNumber("Algae Remover SetPoint", pidSetPoint);
    SmartDashboard.putBoolean("Algae Remover At SetPoint", removerPID.atSetpoint());
  }

  public void teleOp(double speed) {
    removerMotor.set(speed);
  }
  public void stop() {
    //removerPID.
    removerMotor.stopMotor();
  }
  public void autoPosition() {
    removerPID.setSetpoint(pidSetPoint);
    removerPID.setTolerance(200);
    removerMotor.set(MathUtil.clamp(removerPID.calculate(removerEncoder.getPosition()), -0.5, 0.5));
  }
  public void resetEncoder(){
    removerEncoder.setPosition(0);
  }
}

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class subAlgaeProcessor extends SubsystemBase {
  SparkMax armMotor = new SparkMax(Constants.Algae.ArmMotor, SparkMax.MotorType.kBrushless);
  SparkMax intakeMotor = new SparkMax(Constants.Algae.IntakeMotor, SparkMax.MotorType.kBrushless);
  PIDController armPID = new PIDController(0.0001, 0.0, 0.0);
  RelativeEncoder armEncoder = armMotor.getEncoder();
  SparkMaxConfig armConfig = new SparkMaxConfig();  
  SparkMaxConfig intakeConfig = new SparkMaxConfig();
  double setPoint = 0;
  public subAlgaeProcessor() {
    armPID.setTolerance(400);
    armEncoder.setPosition(0);
    armConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    armConfig.encoder
      .positionConversionFactor(1000)
      .velocityConversionFactor(1000);
    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    intakeConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    intakeConfig.encoder 
      .positionConversionFactor(1000)
      .velocityConversionFactor(1000);
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Processor Arm Encoder", armEncoder.getPosition());
    SmartDashboard.putNumber("Processor Arm Setpoint", setPoint);
    SmartDashboard.putBoolean("Processor Arm At Setpoint", armPID.atSetpoint());
  }
  public void setSetPoint(double setPoint) {
    this.setPoint = setPoint;
  }
  public void teleOp_Arm(double speed) {
    armMotor.set(speed);
  }
  public void teleOp_Intake(double speed) {
    intakeMotor.set(speed);
  }
  public void stop() {
    armMotor.stopMotor();
    intakeMotor.stopMotor();
  }
  public void stopArm() {
    armMotor.stopMotor();
  } 
  public void stopIntake() {
    intakeMotor.stopMotor();
  }
  public void moveArmUp(){
    armPID.setSetpoint(Constants.Algae.ArmUpSensorValue);
    armMotor.set(MathUtil.clamp(armPID.calculate(armEncoder.getPosition()), -0.3, 0.3));
  }
  public void moveArmDown(){
    armPID.setSetpoint(Constants.Algae.ArmDownSensorValue);
    armMotor.set(MathUtil.clamp(armPID.calculate(armEncoder.getPosition()), -0.2, 0.2));
  }
}
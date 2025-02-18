package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class subElevator extends SubsystemBase {
  SparkMax lowerMotor = new SparkMax(Constants.Elevator.LowerMotor, SparkMax.MotorType.kBrushless);
  SparkMax upperMotor = new SparkMax(Constants.Elevator.UpperMotor, SparkMax.MotorType.kBrushless);
  RelativeEncoder lowerEncoder = lowerMotor.getEncoder();
  RelativeEncoder upperEncoder = upperMotor.getEncoder();
  SparkMaxConfig lowerConfig = new SparkMaxConfig();
  SparkMaxConfig upperConfig = new SparkMaxConfig();
  DigitalInput lowerLimitFront = new DigitalInput(Constants.Elevator.LowerLimitFront);
  DigitalInput lowerLimitBack = new DigitalInput(Constants.Elevator.LowerLimitBack);
  DigitalInput upperLimitFront = new DigitalInput(Constants.Elevator.UpperLimitFront);
  DigitalInput upperLimitBack = new DigitalInput(Constants.Elevator.UpperLimitBack);
  AnalogPotentiometer stringPotentiometer = new AnalogPotentiometer(Constants.Elevator.StringPot);
  PIDController elevatorPID = new PIDController(0.04, 0.0, 0.0);

  public subElevator() {
    elevatorPID.setTolerance(0.1);
    elevatorPID.setIntegratorRange(-0.1, 0.1);

    lowerEncoder.setPosition(0);
    upperEncoder.setPosition(0);

    lowerConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    lowerConfig.encoder
      .positionConversionFactor(1000)
      .velocityConversionFactor(1000);
    lowerMotor.configure(lowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    upperConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .follow(Constants.Elevator.LowerMotor);
    upperConfig.encoder
      .positionConversionFactor(1000)
      .velocityConversionFactor(1000);
    upperMotor.configure(upperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Height", stringPotentiometer.get());
    SmartDashboard.putBoolean("Elevator Top Limit Front", !upperLimitFront.get());
    SmartDashboard.putBoolean("Elevator Bottom Limit Back", !lowerLimitBack.get());
    SmartDashboard.putBoolean("Elevator Top Limit Back", !upperLimitBack.get());
    SmartDashboard.putBoolean("Elevator Bottom Limit Front", !lowerLimitFront.get());
    SmartDashboard.putBoolean("Elevator Bottom Combined", getLowerLimit());
    SmartDashboard.putBoolean("Elevator Top Combined", getUpperLimit());
  }

  public boolean getLowerLimit() {
    return lowerLimitBack.get() || lowerLimitFront.get() ? false : true;
  }

  public boolean getUpperLimit() {
    return upperLimitBack.get() || upperLimitFront.get() ? false : true;
  }

  public void teleOpNoSafety(double speed) {
    lowerMotor.set(speed);
  }

  public void teleOp(double speed) {
    if(getLowerLimit() && speed < 0) {
      stop();
    }
    else if (getUpperLimit() && speed > 0) {
      stop();
    }
    else{
      lowerMotor.set(speed);
    }    
  }

  public void moveToPosition(double position) {
    if(getLowerLimit() && lowerMotor.get() > 0) {
      stop();
    }
    else if (getUpperLimit() && lowerMotor.get() < 0) {
      stop();
    }
    else{
      lowerMotor.set(elevatorPID.calculate(stringPotentiometer.get(), position));
    }
  }

  public void stop() {
    lowerMotor.stopMotor();
    upperMotor.stopMotor();
  }

  public void resetEncoders() {
    lowerEncoder.setPosition(0);
    upperEncoder.setPosition(0);
  }
}
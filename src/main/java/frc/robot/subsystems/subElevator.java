package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
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
  DigitalInput lowerLimit = new DigitalInput(Constants.Elevator.LowerLimit);
  DigitalInput upperLimit = new DigitalInput(Constants.Elevator.UpperLimit);
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
  }

  public void teleOp(double speed) {
    lowerMotor.set(speed);
  }

  public void autoToPosition(double position) {
    if(lowerLimit.get() && lowerMotor.get() > 0) {
      stop();
    }
    else if (upperLimit.get() && lowerMotor.get() < 0) {
      stop();
    }
    else{
      lowerMotor.set(elevatorPID.calculate(stringPotentiometer.get(), position));
    }
  }

  public void stop() {
    lowerMotor.stopMotor();
  }

  public void resetEncoders() {
    lowerEncoder.setPosition(0);
    upperEncoder.setPosition(0);
  }
}
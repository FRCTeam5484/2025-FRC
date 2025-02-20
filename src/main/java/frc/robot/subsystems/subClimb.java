
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class subClimb extends SubsystemBase {
  SparkMax winchMotor = new SparkMax(Constants.Climb.winchMotor, SparkMax.MotorType.kBrushless);
  SparkMax hookMotor = new SparkMax(Constants.Climb.hookMotor, SparkMax.MotorType.kBrushless);
  public subClimb() {}

  @Override
  public void periodic() {
  }
  public void TeleOpHook(){
    hookMotor.set(0.5);
  }
  public void TeleOpWinch(){
    winchMotor.set(0.5);
  }
  public void StopHook(){
    hookMotor.stopMotor();
  }
  public void StopWinch(){
    winchMotor.stopMotor();
  }
}

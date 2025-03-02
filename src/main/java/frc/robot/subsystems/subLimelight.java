
package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.classes.LimelightHelpers;

public class subLimelight extends SubsystemBase {
  public subLimelight() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
    SmartDashboard.putNumber("Limelight Horizontal Error", getHorizontalError());
    SmartDashboard.putNumber("Limelight Distance Error", getDistanceError());
  }

  public double getHorizontalError(){
    // Horizontal offset from crosshair to target in degrees
    return (LimelightHelpers.getTX("limelight-right") + Constants.LimeLightOffsets.HorizontalOffset);
  }
  public double getDistanceError(){
    // Target area (0% to 100% of image)
    return (LimelightHelpers.getTA("limelight-right")+ Constants.LimeLightOffsets.DistanceOffset);
  }
  public boolean hasTarget(){
    // Do you have a valid target?
    return LimelightHelpers.getTV("limelight-right");
  }
}
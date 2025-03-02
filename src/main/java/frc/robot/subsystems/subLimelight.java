
package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.LimelightHelpers;

public class subLimelight extends SubsystemBase {
  public subLimelight() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
  }

  public double getHorizontalError(double offset){
    // Horizontal offset from crosshair to target in degrees
    return LimelightHelpers.getTX("limelight-right") + offset;
  }
  public double getDistanceError(double offset){
    // Target area (0% to 100% of image)
    return LimelightHelpers.getTA("limelight-right")+ offset;
  }
  public boolean hasTarget(){
    // Do you have a valid target?
    return LimelightHelpers.getTV("limelight-right");
  }
}
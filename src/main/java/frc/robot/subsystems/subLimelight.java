
package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.LimelightHelpers;

public class subLimelight extends SubsystemBase {
  PIDController limePID = new PIDController(0.02, 0, 0);
  public subLimelight() {
    LimelightHelpers.setLEDMode_ForceOff("");
    limePID.setIntegratorRange(-0.2, 0.2);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
    SmartDashboard.putNumber("LimeLight X", getX());
    SmartDashboard.putNumber("LimeLight Y", getY());
    SmartDashboard.putNumber("LimeLight A", getA());
  }

  public double getY(){
    return LimelightHelpers.getTY("limelight")+15.819;
  }
  public double getX(){
    return LimelightHelpers.getTX("limelight")-8.99;
  }
  public double getA(){
    return LimelightHelpers.getTA("limelight")-3.66;
  }
  public boolean hasTarget(){
    return LimelightHelpers.getTV("limelight");
  }
  public double pidCorrection(){
    return -limePID.calculate(getX(), 0);
  }
  public boolean readyToFire(){
    return Math.abs(pidCorrection()) < 1 ? true : false;
  }
  public void setBluePipeline(){
    LimelightHelpers.setPipelineIndex("limelight", 1);
  }
  public void setRedPipeline(){
    LimelightHelpers.setPipelineIndex("limelight", 2);
  }
  public void setDriverPipeline(){
    LimelightHelpers.setPipelineIndex("limelight", 0);
  }
}
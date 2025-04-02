package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.classes.LimelightHelpers;
import frc.robot.subsystems.subSwerve;
import swervelib.SwerveInputStream;

public class cmdAuto_AlignRobotRight extends Command {
  subSwerve swerve;
  PIDController pidHorizontalController = new PIDController(0.09, 0.0, 0.0);
  PIDController pidDistanceController = new PIDController(0.05, 0.0, 0.0);
  SwerveInputStream driveAngularVelocity;

  public cmdAuto_AlignRobotRight(subSwerve swerve) {
    this.swerve = swerve;
    pidHorizontalController.setSetpoint(Constants.LimeLightOffsets.Right.HorizontalOffset);
    pidDistanceController.setSetpoint(Constants.LimeLightOffsets.Right.DistanceOffset);
    pidHorizontalController.setTolerance(1);
    pidDistanceController.setTolerance(2);
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if(!LimelightHelpers.getTV("limelight-right")){
      swerve.drive(new ChassisSpeeds(0, 0, 0));    
      return;
    }
    swerve.drive(new ChassisSpeeds(
      MathUtil.clamp(pidDistanceController.calculate(LimelightHelpers.getTA("limelight-right")), -0.4, 0),
      MathUtil.clamp(pidHorizontalController.calculate(LimelightHelpers.getTX("limelight-right")), -0.9, 0.9), 
      0));     
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(0, 0, 0));    
  }

  @Override
  public boolean isFinished() {
    return pidHorizontalController.atSetpoint() && pidDistanceController.atSetpoint() ? true : false;
  }
}

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.classes.LimelightHelpers;
import frc.robot.subsystems.subSwerve;
import swervelib.SwerveInputStream;

public class cmdAuto_AlignRobot extends Command {
  subSwerve swerve;
  PIDController pidHorizontalController = new PIDController(0.9, 0.0, 0.0);
  PIDController pidDistanceController = new PIDController(0.5, 0.0, 0.0);
  SwerveInputStream driveAngularVelocity;

  public cmdAuto_AlignRobot(subSwerve swerve) {
    this.swerve = swerve;
    pidHorizontalController.setSetpoint(Constants.LimeLightOffsets.HorizontalOffset);
    pidDistanceController.setSetpoint(Constants.LimeLightOffsets.DistanceOffset);
    pidHorizontalController.setTolerance(5);
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
      MathUtil.clamp(-pidDistanceController.calculate(LimelightHelpers.getTA("limelight-right")), -0.4, 0),
      MathUtil.clamp(-pidHorizontalController.calculate(LimelightHelpers.getTX("limelight-right")), -0.6, 0.6), 
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

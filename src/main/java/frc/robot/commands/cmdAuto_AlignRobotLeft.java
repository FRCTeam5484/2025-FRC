package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.classes.LimelightHelpers;
import frc.robot.subsystems.subSwerve;
import swervelib.SwerveInputStream;

public class cmdAuto_AlignRobotLeft extends Command {
  subSwerve swerve;
  PIDController pidHorizontalController = new PIDController(0.0005, 0.0, 0.0);
  PIDController pidDistanceController = new PIDController(0.04, 0.0, 0.0);
  SwerveInputStream driveAngularVelocity;

  public cmdAuto_AlignRobotLeft(subSwerve swerve) {
    this.swerve = swerve;
    pidHorizontalController.setSetpoint(Constants.LimeLightOffsets.Left.HorizontalOffset*100);
    pidDistanceController.setSetpoint(Constants.LimeLightOffsets.Left.DistanceOffset*100);
    pidHorizontalController.setTolerance(10);
    pidDistanceController.setTolerance(10);
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    System.out.println("Left Limelight: " + LimelightHelpers.getTV("limelight-left"));
    if(!LimelightHelpers.getTV("limelight-left")){
      swerve.drive(new ChassisSpeeds(0, 0, 0));    
      return;
    }
    double horizontalCommand = MathUtil.clamp(pidHorizontalController.calculate(LimelightHelpers.getTX("limelight-left")*100), -0.9, 0.9);
    //System.out.println(horizontalCommand);
    swerve.drive(new ChassisSpeeds(
      MathUtil.clamp(-pidDistanceController.calculate(LimelightHelpers.getTA("limelight-left")*100), 0, 0.4),
      horizontalCommand, 
      0));     
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(0, 0, 0));    
  }

  @Override
  public boolean isFinished() {
    return pidHorizontalController.atSetpoint();
  }
}

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.subLimelight;
import frc.robot.subsystems.subSwerve;
import swervelib.SwerveInputStream;

public class cmdAuto_AlignRobot extends Command {
  subSwerve swerve;
  subLimelight limelight;
  PIDController pidHorizontalController = new PIDController(0.05, 0.0, 0.0);
  PIDController pidDistanceController = new PIDController(0.05, 0.0, 0.0);
  double horizontalError = 0;
  double distanceError = 0;
  SwerveInputStream driveAngularVelocity;
  double forwardCommand = 0;
  double strafeCommand = 0;

  public cmdAuto_AlignRobot(subSwerve swerve, subLimelight lime) {
    this.swerve = swerve;
    this.limelight = lime;
    pidHorizontalController.setSetpoint(0);
    pidDistanceController.setSetpoint(0);
    
    addRequirements(swerve, limelight);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if(!limelight.hasTarget()){
      swerve.drive(new ChassisSpeeds(0, 0, 0));    
      return;
    }
    horizontalError = limelight.getHorizontalError();
    distanceError = limelight.getDistanceError();
    forwardCommand = MathUtil.clamp(pidDistanceController.calculate(distanceError), -1, 1);
    strafeCommand = MathUtil.clamp(pidHorizontalController.calculate(horizontalError), -1, 1);
    //System.out.println("Horizontal Error: " + strafeCommand + " Distance Error: " + horizontalError);
    swerve.drive(new ChassisSpeeds(-forwardCommand, -strafeCommand, 0));    
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

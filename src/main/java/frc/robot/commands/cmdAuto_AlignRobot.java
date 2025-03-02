package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.subLimelight;
import frc.robot.subsystems.subSwerve;

public class cmdAuto_AlignRobot extends Command {
  subSwerve swerve;
  subLimelight limelight;
  String level;
  PIDController pidHorizontalController = new PIDController(0.001, 0.0, 0.0);
  PIDController pidDistanceController = new PIDController(0.001, 0.0, 0.0);
  double horizontalError = 0;
  double distanceError = 0;
  boolean hasTarget;
  boolean cancelCommand = false;

  public cmdAuto_AlignRobot(subSwerve swerve, subLimelight lime, String level) {
    this.swerve = swerve;
    this.limelight = lime;
    this.level = level;
    pidHorizontalController.setSetpoint(0);
    pidDistanceController.setSetpoint(0);
    pidHorizontalController.setTolerance(1);
    pidDistanceController.setTolerance(1);
    addRequirements(swerve, limelight);
  }

  @Override
  public void initialize() {
    hasTarget = limelight.hasTarget();
    if(!hasTarget){
      cancelCommand = true;
    }
  }

  @Override
  public void execute() {
    if(cancelCommand){
      return;
    }
    if(!hasTarget){
      cancelCommand = true;
      return;
    }
    if(level == "L4"){
      horizontalError = limelight.getHorizontalError(Constants.LimeLightOffsets.L4HorizontalOffset);
      distanceError = limelight.getDistanceError(Constants.LimeLightOffsets.L4DistanceOffset);
    } 
    else if(level == "L3"){
      horizontalError = limelight.getHorizontalError(Constants.LimeLightOffsets.L3HorizontalOffset);
      distanceError = limelight.getDistanceError(Constants.LimeLightOffsets.L3DistanceOffset);
    }
    else if(level == "L2"){
      horizontalError = limelight.getHorizontalError(Constants.LimeLightOffsets.L2HorizontalOffset);
      distanceError = limelight.getDistanceError(Constants.LimeLightOffsets.L2DistanceOffset);
    }
    else if(level == "L1"){
      horizontalError = limelight.getHorizontalError(Constants.LimeLightOffsets.L1HorizontalOffset);
      distanceError = limelight.getDistanceError(Constants.LimeLightOffsets.L1DistanceOffset);
    }
    swerve.driveCommand(
      ()->MathUtil.clamp(pidDistanceController.calculate(distanceError), -0.3, 0.3),
      ()->MathUtil.clamp(pidHorizontalController.calculate(horizontalError), -0.3, 0.3),
      ()->0);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.driveCommand(()->0, ()->0, ()->0);
  }

  @Override
  public boolean isFinished() {
    return cancelCommand || pidHorizontalController.atSetpoint() && pidDistanceController.atSetpoint() ? true : false;
  }
}

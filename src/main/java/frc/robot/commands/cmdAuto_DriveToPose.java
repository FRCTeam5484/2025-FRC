package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subSwerve;

public class cmdAuto_DriveToPose extends Command {
  subSwerve swerve;
  Pose2d pose;
  public cmdAuto_DriveToPose(subSwerve swerve, Pose2d pose) {
    this.swerve = swerve;
    this.pose = pose;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    swerve.driveToPose(pose);
  }

  @Override
  public void end(boolean interrupted) {
    //swerve.drive(new ChassisSpeeds(0, 0, 0));  
  }

  @Override
  public boolean isFinished() {
    return false;//swerve.getPose().getTranslation().getDistance(pose.getTranslation()) < 0.1;
  }
}

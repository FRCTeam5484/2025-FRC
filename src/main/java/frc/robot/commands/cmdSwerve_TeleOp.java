
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subSwerve;
import swervelib.SwerveInputStream;

public class cmdSwerve_TeleOp extends Command {
  SwerveInputStream directAngle;
  subSwerve swerve;
  public cmdSwerve_TeleOp(subSwerve swerve, SwerveInputStream directAngle) {
    this.swerve = swerve;
    this.directAngle = directAngle;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    swerve.driveFieldOrientated(directAngle);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}

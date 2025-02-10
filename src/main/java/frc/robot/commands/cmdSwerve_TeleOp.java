package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.subSwerve;

public class cmdSwerve_TeleOp extends Command {
  private final subSwerve swerve;
  private final DoubleSupplier xSupplier, ySupplier, rotSupplier;
  public cmdSwerve_TeleOp(subSwerve swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier) {
    this.swerve = swerve;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotSupplier = rotSupplier;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    swerve.drive(
        xSupplier.getAsDouble() * Constants.DriveConstants.kMaxSpeedMetersPerSecond, 
        ySupplier.getAsDouble() * Constants.DriveConstants.kMaxSpeedMetersPerSecond, 
        rotSupplier.getAsDouble() * Constants.DriveConstants.kMaxAngularSpeed
      );
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

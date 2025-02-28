package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subSwerve;

public class cmdSwerve_TeleOpAlt extends Command {
  private final subSwerve swerve;
  private final DoubleSupplier  vX, vY, heading;
  private final BooleanSupplier fieldCentric;
  public cmdSwerve_TeleOpAlt(subSwerve swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading, BooleanSupplier fieldCentric) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;
    this.fieldCentric = fieldCentric;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    swerve.driveCommand(vY, vX, heading, fieldCentric);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subCoral;

public class cmdCoral_Stop extends Command {
  subCoral coral;
  public cmdCoral_Stop(subCoral coral) {
    this.coral = coral;
    addRequirements(coral);
  }

  @Override
  public void initialize() {
    coral.stop();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subCoral;

public class cmdAuto_CoralEject extends Command {
  subCoral coral;
  public cmdAuto_CoralEject(subCoral coral) {
    this.coral = coral;
    addRequirements(coral);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    coral.teleOp(0.8);
  }

  @Override
  public void end(boolean interrupted) {
    coral.stop();
  }

  @Override
  public boolean isFinished() {
    return !coral.InSensor()  && !coral.OutSensor();
  }
}

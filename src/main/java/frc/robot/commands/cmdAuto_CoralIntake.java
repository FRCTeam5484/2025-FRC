
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subBlinkin;
import frc.robot.subsystems.subCoral;

public class cmdAuto_CoralIntake extends Command {
  subCoral coral;
  public cmdAuto_CoralIntake(subCoral coral) {
    this.coral = coral;
    addRequirements(coral);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    coral.teleOp(0.5);
  }

  @Override
  public void end(boolean interrupted) {
    coral.stop();
  }

  @Override
  public boolean isFinished() {
    return coral.hasCoral();
  }
}

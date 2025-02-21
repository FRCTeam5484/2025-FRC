
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subAlgaeRemover;

public class cmdAlgaeRemover_ResetEncoder extends Command {
  subAlgaeRemover remover;
  public cmdAlgaeRemover_ResetEncoder(subAlgaeRemover remover) {
    this.remover = remover;
    addRequirements(remover);
  }

  @Override
  public void initialize() {
    remover.resetEncoder();
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

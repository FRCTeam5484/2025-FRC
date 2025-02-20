
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subAlgaeRemover;

public class cmdAlgaeRemover_AutoPosition extends Command {
  subAlgaeRemover remover;
  Double position;
  public cmdAlgaeRemover_AutoPosition(subAlgaeRemover remover, Double position) {
    this.remover = remover;
    this.position = position;
    addRequirements(remover);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    remover.autoPosition(position);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}

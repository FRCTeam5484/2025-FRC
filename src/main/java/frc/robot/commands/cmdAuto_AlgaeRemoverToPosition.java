
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subAlgaeRemover;

public class cmdAuto_AlgaeRemoverToPosition extends Command {
  subAlgaeRemover remover;
  Double position;
  public cmdAuto_AlgaeRemoverToPosition(subAlgaeRemover remover, Double position) {
    this.remover = remover;
    this.position = position;
    addRequirements(remover);
  }

  @Override
  public void initialize() {
    remover.pidSetPoint = position;
  }

  @Override
  public void execute() {
    remover.autoPosition();
  }

  @Override
  public void end(boolean interrupted) {
    remover.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

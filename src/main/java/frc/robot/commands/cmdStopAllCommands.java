package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subAlgaeRemover;
import frc.robot.subsystems.subCoral;
import frc.robot.subsystems.subElevator;

public class cmdStopAllCommands extends Command {
  subAlgaeRemover remover;
  subCoral coral; 
  subElevator elevator;
  public cmdStopAllCommands(subAlgaeRemover remover, subCoral coral, subElevator elevator) {
    this.remover = remover;
    this.coral = coral;
    this.elevator = elevator;
    addRequirements(remover, coral, elevator);
  }

  @Override
  public void initialize() {
    remover.stop();
    coral.stop();
    elevator.stop();
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

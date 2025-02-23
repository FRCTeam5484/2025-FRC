package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subAlgaeProcessor;
import frc.robot.subsystems.subAlgaeRemover;
import frc.robot.subsystems.subBlinkin;
import frc.robot.subsystems.subCoral;
import frc.robot.subsystems.subElevator;

public class cmdStopAllCommands extends Command {
  subAlgaeProcessor processor;
  subAlgaeRemover remover;
  subCoral coral; 
  subElevator elevator;
  subBlinkin blinkin;
  public cmdStopAllCommands(subAlgaeProcessor processor, subAlgaeRemover remover, subCoral coral, subElevator elevator, subBlinkin blinkin) {
    this.processor = processor;
    this.remover = remover;
    this.coral = coral;
    this.elevator = elevator;
    this.blinkin = blinkin;
    addRequirements(processor, remover, coral, elevator, blinkin);
  }

  @Override
  public void initialize() {
    processor.stop();
    remover.stop();
    coral.stop();
    elevator.stop();
    blinkin.off();
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

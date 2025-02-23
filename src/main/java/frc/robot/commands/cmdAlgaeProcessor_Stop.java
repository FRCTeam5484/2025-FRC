package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subAlgaeProcessor;

public class cmdAlgaeProcessor_Stop extends Command {
  subAlgaeProcessor processor;
  public cmdAlgaeProcessor_Stop(subAlgaeProcessor processor) {
    this.processor = processor;
    addRequirements(processor);
  }

  @Override
  public void initialize() {
    processor.stop();
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

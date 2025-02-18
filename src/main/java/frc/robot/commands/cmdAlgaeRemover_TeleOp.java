package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subAlgaeRemover;

public class cmdAlgaeRemover_TeleOp extends Command {
  subAlgaeRemover remover;
  DoubleSupplier speed;
  public cmdAlgaeRemover_TeleOp(subAlgaeRemover remover, DoubleSupplier speed) {
    this.remover = remover;
    this.speed = speed;
    addRequirements(remover);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    remover.teleOp(speed.getAsDouble());
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

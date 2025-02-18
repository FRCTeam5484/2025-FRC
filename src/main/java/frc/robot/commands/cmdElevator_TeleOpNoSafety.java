package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subElevator;

public class cmdElevator_TeleOpNoSafety extends Command {
  subElevator elevator;
  DoubleSupplier speed;
  public cmdElevator_TeleOpNoSafety(subElevator elevator, DoubleSupplier speed) {
    this.elevator = elevator;
    this.speed = speed;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevator.teleOpNoSafety(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

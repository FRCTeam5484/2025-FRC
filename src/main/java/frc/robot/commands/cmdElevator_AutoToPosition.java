package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subElevator;

public class cmdElevator_AutoToPosition extends Command {
  subElevator elevator;
  double position;
  public cmdElevator_AutoToPosition(subElevator elevator, double position) {
    this.elevator = elevator;
    this.position = position;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.setPoint = position;
  }

  @Override
  public void execute() {
    elevator.moveToPosition();
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

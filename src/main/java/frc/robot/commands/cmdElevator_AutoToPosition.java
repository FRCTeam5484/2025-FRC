package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subBlinkin;
import frc.robot.subsystems.subElevator;

public class cmdElevator_AutoToPosition extends Command {
  subElevator elevator;
  subBlinkin blinkin;
  double position;
  public cmdElevator_AutoToPosition(subElevator elevator, subBlinkin blinkin, double position) {
    this.elevator = elevator;
    this.blinkin = blinkin;
    this.position = position;
    addRequirements(elevator, blinkin);
  }

  @Override
  public void initialize() {
    blinkin.strobeBlue();
    elevator.setPoint = position;
  }

  @Override
  public void execute() {
    if(elevator.atSetPoint()) {
      blinkin.green();
    }
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

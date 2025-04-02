
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.subCoral;
import frc.robot.subsystems.subElevator;

public class cmdAuto_CoralIntakeElevatorL2 extends Command {
  subCoral coral;
  subElevator elevator;
  public cmdAuto_CoralIntakeElevatorL2(subCoral coral, subElevator elevator) {
    this.coral = coral;
    this.elevator = elevator;
    addRequirements(coral, elevator);
  }

  @Override
  public void initialize() {
    elevator.setPoint = Constants.Elevator.bottomPosition;
  }

  @Override
  public void execute() {
    if(!coral.hasCoral()){
      coral.teleOp(0.5);
    }
    else{
      elevator.setPoint = Constants.Elevator.L2;
      coral.stop();
    }    
    elevator.moveToPosition();
  }

  @Override
  public void end(boolean interrupted) {
    coral.stop();
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

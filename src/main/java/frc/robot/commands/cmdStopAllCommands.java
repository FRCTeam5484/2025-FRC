package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subAlgaeArm;
import frc.robot.subsystems.subAlgaeIntake;
import frc.robot.subsystems.subAlgaeRemover;
import frc.robot.subsystems.subCoral;
import frc.robot.subsystems.subElevator;

public class cmdStopAllCommands extends Command {
  subAlgaeArm arm;
  subAlgaeIntake intake;
  subAlgaeRemover remover;
  subCoral coral;
  subElevator elevator;
  public cmdStopAllCommands(subAlgaeArm arm, subAlgaeIntake intake, subAlgaeRemover remover, subCoral coral, subElevator elevator) {
    this.arm = arm;
    this.intake = intake;
    this.remover = remover;
    this.coral = coral;
    this.elevator = elevator;
    addRequirements(arm, intake, remover, coral, elevator);
  }

  @Override
  public void initialize() {
    arm.stop();
    intake.stop();
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

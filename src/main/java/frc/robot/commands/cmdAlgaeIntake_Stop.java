package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subAlgae;

public class cmdAlgaeIntake_Stop extends Command {
  subAlgae algae;
  public cmdAlgaeIntake_Stop(subAlgae algae) {
    this.algae = algae;
    addRequirements(algae);
  }

  @Override
  public void initialize() {
    algae.stopIntake();
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

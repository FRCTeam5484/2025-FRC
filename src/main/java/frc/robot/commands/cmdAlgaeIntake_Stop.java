package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subAlgaeIntake;

public class cmdAlgaeIntake_Stop extends Command {
  subAlgaeIntake algae;
  public cmdAlgaeIntake_Stop(subAlgaeIntake algae) {
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

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subAlgaeArm;

public class cmdAlgaeArm_Stop extends Command {
  subAlgaeArm algae;
  public cmdAlgaeArm_Stop(subAlgaeArm algae) {
    this.algae = algae;
    addRequirements(algae);
  }

  @Override
  public void initialize() {
    algae.stopArm();
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

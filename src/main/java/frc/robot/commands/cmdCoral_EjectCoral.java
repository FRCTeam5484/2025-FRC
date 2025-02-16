package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subCoral;

public class cmdCoral_EjectCoral extends Command {
  subCoral coral;
  Timer timer = new Timer();
  public cmdCoral_EjectCoral(subCoral coral) {
    this.coral = coral;
    addRequirements(coral);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    coral.teleOp(0.5);
  }

  @Override
  public void end(boolean interrupted) {
    coral.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.get() > 2;
  }
}

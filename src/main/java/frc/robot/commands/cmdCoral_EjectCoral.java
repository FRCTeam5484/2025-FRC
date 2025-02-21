package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subBlinkin;
import frc.robot.subsystems.subCoral;

public class cmdCoral_EjectCoral extends Command {
  subCoral coral;
  subBlinkin blinkin;
  Timer timer = new Timer();
  public cmdCoral_EjectCoral(subCoral coral, subBlinkin blinkin) {
    this.coral = coral;
    this.blinkin = blinkin;
    addRequirements(coral, blinkin);
  }

  @Override
  public void initialize() {
    blinkin.strobeRed();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    coral.teleOp(0.6);
  }

  @Override
  public void end(boolean interrupted) {
    blinkin.green();
    coral.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.get() > 1.5;
  }
}

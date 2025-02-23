
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subBlinkin;
import frc.robot.subsystems.subCoral;

public class cmdAuto_CoralIntake extends Command {
  subCoral coral;
  subBlinkin blinkin;
  public cmdAuto_CoralIntake(subCoral coral, subBlinkin blinkin) {
    this.coral = coral;
    this.blinkin = blinkin;
    addRequirements(coral, blinkin);
  }

  @Override
  public void initialize() {
    blinkin.strobeBlue();
  }

  @Override
  public void execute() {
    coral.teleOp(0.5);
  }

  @Override
  public void end(boolean interrupted) {
    blinkin.green();
    coral.stop();
  }

  @Override
  public boolean isFinished() {
    return coral.hasCoral();
  }
}

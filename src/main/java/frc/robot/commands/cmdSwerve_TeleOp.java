
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subSwerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class cmdSwerve_TeleOp extends Command {
  private final subSwerve swerve;
  private final DoubleSupplier  vX, vY, heading;
  private final BooleanSupplier fieldCentric;
  Command fieldCentricMode;
  Command robotCentricMode;
  public cmdSwerve_TeleOp(subSwerve swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading, BooleanSupplier fieldCentric)
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;
    this.fieldCentric = fieldCentric;
    fieldCentricMode = new cmdSwerve_TeleOpFieldCentricMode(this.swerve, this.vX, this.vY, this.heading);
    robotCentricMode = new cmdSwerve_TeleOpRobotCentricMode(this.swerve, this.vX, this.vY, this.heading);
    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (fieldCentric.getAsBoolean()) {
      fieldCentricMode.execute();
    } else {
      robotCentricMode.execute();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
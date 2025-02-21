package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.subAlgaeArm;
import frc.robot.subsystems.subAlgaeIntake;

public class cmdAlgae_AutoHold extends Command {
  subAlgaeArm arm;
  subAlgaeIntake intake;
  public cmdAlgae_AutoHold(subAlgaeArm arm, subAlgaeIntake intake) {
    this.arm = arm;
    this.intake = intake;
    addRequirements(arm, intake);
  }

  @Override
  public void initialize() {
    arm.setPoint = Constants.Algae.ArmUpSensorValue;
  }

  @Override
  public void execute() {
    arm.moveToPosition();
    intake.autoHold();
  }

  @Override
  public void end(boolean interrupted) {
    arm.stop();
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

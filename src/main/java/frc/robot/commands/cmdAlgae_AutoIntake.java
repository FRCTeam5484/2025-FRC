package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subAlgaeArm;
import frc.robot.subsystems.subAlgaeIntake;

public class cmdAlgae_AutoIntake extends Command {
  subAlgaeArm arm;
  subAlgaeIntake intake;

  public cmdAlgae_AutoIntake(subAlgaeArm arm, subAlgaeIntake intake) {
    this.arm = arm;
    this.intake = intake;
    addRequirements(arm, intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    arm.autoDownPosition();
    intake.autoIntake();
  }

  @Override
  public void end(boolean interrupted) {  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

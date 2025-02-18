package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subAlgaeArm;
import frc.robot.subsystems.subAlgaeIntake;

public class cmdAlgae_AutoEject extends Command {
  subAlgaeArm arm;
  subAlgaeIntake intake;
  Timer timer = new Timer();
  public cmdAlgae_AutoEject(subAlgaeArm arm, subAlgaeIntake intake) {
    this.arm = arm;
    this.intake = intake; 
    addRequirements(arm, intake);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    intake.teleOp(-1);
    arm.autoUpPosition();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    arm.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.get() > 1 ? true : false;
  }
}

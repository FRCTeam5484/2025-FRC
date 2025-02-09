package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subAlgae;

public class cmdAlgaeIntake_TeleOp extends Command {
  subAlgae algae;
  DoubleSupplier speed;
  public cmdAlgaeIntake_TeleOp(subAlgae algae, DoubleSupplier speed) {
    this.algae = algae;
    this.speed = speed;
    addRequirements(algae);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    algae.teleOp_Intake(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    algae.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

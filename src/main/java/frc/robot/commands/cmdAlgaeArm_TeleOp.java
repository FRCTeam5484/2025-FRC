package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subAlgae;

public class cmdAlgaeArm_TeleOp extends Command {
  subAlgae algae;
  DoubleSupplier speed;
  public cmdAlgaeArm_TeleOp(subAlgae algae, DoubleSupplier speed) {
    this.algae = algae;
    this.speed = speed;
    addRequirements(algae);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    algae.teleOp_Arm(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    algae.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

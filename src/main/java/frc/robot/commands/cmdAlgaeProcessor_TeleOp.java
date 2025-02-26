package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subAlgaeProcessor;

public class cmdAlgaeProcessor_TeleOp extends Command {
  subAlgaeProcessor algaeProcessor;
  DoubleSupplier speed;
  public cmdAlgaeProcessor_TeleOp(subAlgaeProcessor processor, DoubleSupplier speed) {
    this.algaeProcessor = processor;
    this.speed = speed;
    addRequirements(algaeProcessor);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    algaeProcessor.teleOp_Arm(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    algaeProcessor.stopArm();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

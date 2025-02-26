package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.subAlgaeProcessor;

public class cmdAuto_AlgaeHold extends Command {
  subAlgaeProcessor processor;
  public cmdAuto_AlgaeHold(subAlgaeProcessor processor) {
    this.processor = processor;
    addRequirements(processor);
  }

  @Override
  public void initialize() {
    processor.setSetPoint(Constants.Algae.ArmUpSensorValue);
  }

  @Override
  public void execute() {
    processor.moveArmUp();
    processor.teleOp_Intake(0.3);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}

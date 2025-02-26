package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.subAlgaeProcessor;

public class cmdAuto_AlgaeIntake extends Command {
  subAlgaeProcessor processor;
  public cmdAuto_AlgaeIntake(subAlgaeProcessor processor) {
    this.processor = processor;
    addRequirements(processor);
  }

  @Override
  public void initialize() { 
    processor.setSetPoint(Constants.Algae.ArmDownSensorValue);
  }
  @Override
  public void execute() {
    processor.moveArmDown();
    processor.teleOp_Intake(0.6);
  }

  @Override
  public void end(boolean interrupted) { 
    processor.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

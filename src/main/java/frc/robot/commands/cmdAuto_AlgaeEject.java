package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.subAlgaeProcessor;

public class cmdAuto_AlgaeEject extends Command {
  subAlgaeProcessor processor;
  Timer timer = new Timer();
  public cmdAuto_AlgaeEject(subAlgaeProcessor processor) {
    this.processor = processor;
    addRequirements(processor);
  }

  @Override
  public void initialize() {
    processor.setSetPoint(Constants.Algae.ArmUpSensorValue);
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    processor.teleOp_Intake(-1);
    processor.moveArmUp();
  }

  @Override
  public void end(boolean interrupted) {
    processor.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.get() > 1 ? true : false;
  }
}

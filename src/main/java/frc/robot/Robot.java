package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.blinkin.teamColorsWaves();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    //m_robotContainer.swerve.zeroGyroWithAlliance();
    m_robotContainer.swerve.zeroGyro();
    m_robotContainer.blinkin.off();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {  }

  @Override
  public void teleopInit() {
    m_robotContainer.blinkin.off();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //m_robotContainer.algaeRemover.pidSetPoint = Constants.Algae.RemoverArmUp;
    //m_robotContainer.algaeRemover.autoPosition();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
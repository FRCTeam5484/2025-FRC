package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.classes.Blinkin;
import frc.robot.classes.LimelightHelpers;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("Limelight Has Target", LimelightHelpers.getTV("limelight-right"));
    SmartDashboard.putNumber("Limelight Horizontal Error", LimelightHelpers.getTX("limelight-right")-Constants.LimeLightOffsets.HorizontalOffset);
    SmartDashboard.putNumber("Limelight Distance Error", LimelightHelpers.getTA("limelight-right")-Constants.LimeLightOffsets.DistanceOffset);
  }

  @Override
  public void disabledInit() {
    Blinkin.teamColorsWaves();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    Blinkin.off();
    m_robotContainer.swerve.zeroGyro();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    if(LimelightHelpers.getTV("limelight-right")){
      Blinkin.strobeBlue();
    }
    else{
      Blinkin.red();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // No Coral and Motor Not Running
    if(!m_robotContainer.coral.hasCoral() && !m_robotContainer.coral.motorRunning()){
      Blinkin.red();
    // No Coral and Motor Running
    } else if(!m_robotContainer.coral.hasCoral() && m_robotContainer.coral.motorRunning()){
      Blinkin.strobeRed();
    // Coral and Motor Not Running
    } else if(m_robotContainer.coral.hasCoral() && !m_robotContainer.coral.motorRunning()){
      Blinkin.green();
    } else {
      Blinkin.off();
    }
  }

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
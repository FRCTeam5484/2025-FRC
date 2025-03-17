package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.subSwerve;
import java.util.List;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class cmdAutonomous_Crossline extends Command {
  private final subSwerve swerve;
  private Timer timer;
  public cmdAutonomous_Crossline(subSwerve swerve) {
    this.swerve = swerve;
    this.timer = new Timer();
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(-0.8, 0, new Rotation2d(0));

    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                           Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                                           swerve.getSwerveDriveConfiguration());

    // Make the robot move
    swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, false);
  }

  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(0, 0, new Rotation2d(0));

    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                           Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                                           swerve.getSwerveDriveConfiguration());

    // Make the robot move
    swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, false);
  }

  @Override
  public boolean isFinished() {
    return timer.get() > 1 ? true : false;
  }
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Meter;

public class subSwerve extends SubsystemBase {
  private final double kMaxSpeed = Units.feetToMeters(19.3);
  private final SwerveDrive swerveDrive;
  private final File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  public subSwerve() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(kMaxSpeed,
                                                                  new Pose2d(new Translation2d(Meter.of(1),
                                                                                               Meter.of(4)),
                                                                             Rotation2d.fromDegrees(0)));  
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  @Override
  public void periodic() {
    /* SwerveModule[] modules = swerveDrive.getModules();
    SmartDashboard.putNumber("Front Left CanCoder", modules[0].getAbsolutePosition());
    SmartDashboard.putNumber("Front Right CanCoder", modules[1].getAbsolutePosition());
    SmartDashboard.putNumber("Back Left CanCoder", modules[2].getAbsolutePosition());
    SmartDashboard.putNumber("Back Right CanCoder", modules[3].getAbsolutePosition()); */
  }
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void driveFieldOrientated(ChassisSpeeds speeds) {
    swerveDrive.driveFieldOriented(speeds);
  }

  public Command driveFieldOrientated(Supplier<ChassisSpeeds> chassisSpeeds) {
    return run(() -> {  
      driveFieldOrientated(chassisSpeeds.get());
    });
  }

  public SwerveDriveKinematics getKinematics(){
    return swerveDrive.kinematics;
  }

  public void resetOdometry(Pose2d initialHolonomicPose){
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void postTrajectory(Trajectory trajectory){
    swerveDrive.postTrajectory(trajectory);
  }

  public void zeroGyro(){
    swerveDrive.zeroGyro();
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading(){
    return getPose().getRotation();
  }
}
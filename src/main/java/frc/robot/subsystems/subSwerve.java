package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.classes.swerveModule;

public class subSwerve extends SubsystemBase {
  
  private final swerveModule frontLeftModule = new swerveModule(SwerveConstants.kFrontLeftDrivingCanId,SwerveConstants.kFrontLeftTurningCanId,SwerveConstants.kFrontLeftCANcoder,SwerveConstants.kFrontLeftOffset);
  private final swerveModule frontRightModule = new swerveModule(SwerveConstants.kFrontRightDrivingCanId,SwerveConstants.kFrontRightTurningCanId,SwerveConstants.kFrontRightCANcoder,SwerveConstants.kFrontRightOffset);
  private final swerveModule rearLeftModule = new swerveModule(SwerveConstants.kRearLeftDrivingCanId,SwerveConstants.kRearLeftTurningCanId,SwerveConstants.kRearLeftCANcoder,SwerveConstants.kRearLeftOffset);
  private final swerveModule rearRightModule = new swerveModule(SwerveConstants.kRearRightDrivingCanId,SwerveConstants.kRearRightTurningCanId,SwerveConstants.kRearRightCANcoder,SwerveConstants.kRearRightOffset);

  private final Pigeon2 gyro;
  public SwerveDriveOdometry odometry;

  public subSwerve() {
    gyro = new Pigeon2(SwerveConstants.Pigeon2);
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.setYaw(0);
    odometry = new SwerveDriveOdometry(
      SwerveConstants.kDriveKinematics,
      gyro.getRotation2d(),
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        rearLeftModule.getPosition(),
        rearRightModule.getPosition()
      });

    /* AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetPose,
      this::getChassisSpeeds,
      this::driveRobotRelative,
      new HolonomicPathFollowerConfig( 
              new PIDConstants(1.0, 0.0, 0.0),
              new PIDConstants(1.0, 0.0, 0.0),
              4.5,
              0.4,
              new ReplanningConfig()
      ),
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    ); */
  }

  public void updateOdometry(){
    odometry.update(
      gyro.getRotation2d(),
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        rearLeftModule.getPosition(),
        rearRightModule.getPosition()
      });
  }
  
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    rearLeftModule.setDesiredState(desiredStates[2]);
    rearRightModule.setDesiredState(desiredStates[3]);
  }

  public void drive(double xSpeed, double ySpeed, double rot) { 
    setModuleStates(SwerveConstants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d()))); 
  }
  public void stopModules(){ 
    frontLeftModule.stopModule(); frontRightModule.stopModule(); rearLeftModule.stopModule(); rearRightModule.stopModule(); 
  }
  public void zeroHeading() { gyro.setYaw(0); }
  public Rotation2d getRotation2d() { return gyro.getRotation2d(); }
  /* public void setGryoOffset(Angle offset){
    Angle correction = gyro.getYaw().getValue();
    gyro.setYaw(correction.plus(offset));
  } */

  // Methods for PathPlanner
  public Pose2d getPose() { return odometry.getPoseMeters(); }
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
      getRotation2d(),
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        rearLeftModule.getPosition(),
        rearRightModule.getPosition()
      },
      pose);
  }
  public ChassisSpeeds getChassisSpeeds(){ 
    return SwerveConstants.kDriveKinematics.toChassisSpeeds(
      frontLeftModule.getState(), 
      frontRightModule.getState(), 
      rearLeftModule.getState(), 
      rearRightModule.getState());
  }
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) { 
    this.drive(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond); 
  }

  @Override
  public void periodic() {
    updateOdometry();
  }
}

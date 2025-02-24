package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subSwerve;
import swervelib.SwerveInputStream;

public class cmdSwerve_TeleOp extends Command {
  subSwerve swerve;
  SwerveInputStream driveAngularVelocity;
  SwerveInputStream driveDirectAngle;
  DoubleSupplier leftX;
  DoubleSupplier leftY;
  DoubleSupplier rightX;
  DoubleSupplier rightY;
  public cmdSwerve_TeleOp(subSwerve swerve, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX, DoubleSupplier rightY) {
    this.swerve = swerve;
    this.leftX = leftX;
    this.leftY = leftY;
    this.rightX = rightX;
    this.rightY = rightY;
    addRequirements(swerve);
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(this.swerve.getSwerveDrive(), 
                                                                () -> leftY.getAsDouble() * -1, 
                                                                () -> leftX.getAsDouble() * -1)
                                                                .withControllerRotationAxis(rightX::getAsDouble)
                                                                .deadband(0.01)
                                                                .scaleTranslation(0.5)
                                                                .allianceRelativeControl(true);

    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(rightX::getAsDouble,
                                                                                            rightY::getAsDouble)
                                                                                           .headingWhile(true);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    swerve.driveFieldOrientated(driveDirectAngle);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}

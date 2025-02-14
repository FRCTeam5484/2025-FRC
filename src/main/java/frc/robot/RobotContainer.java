package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.cmdAlgaeArm_TeleOp;
import frc.robot.commands.cmdAlgaeIntake_TeleOp;
import frc.robot.commands.cmdCoral_TeleOp;
import frc.robot.commands.cmdElevator_TeleOp;
import frc.robot.subsystems.subSwerve;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.subAlgae;
import frc.robot.subsystems.subCoral;
import frc.robot.subsystems.subElevator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
  private final subCoral coral = new subCoral();
  private final subAlgae algae = new subAlgae();
  private final subSwerve swerve = new subSwerve();
  private final subElevator elevator = new subElevator();
  private final CommandXboxController driverOne = new CommandXboxController(OperatorConstants.DriverOne);
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(), 
                                                                () -> driverOne.getLeftY() * -1, 
                                                                () -> driverOne.getLeftX() * -1)
                                                                .withControllerRotationAxis(driverOne::getRightX)
                                                                .deadband(0.05)
                                                                .scaleTranslation(0.5)
                                                                .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverOne::getRightX,
                                                                                            driverOne::getRightY)
                                                                                           .headingWhile(true);
  
  public RobotContainer() {
    //DriverOneControls();
    //DriverTwoControls();
    SingleDriverControls();
  }

  private void DriverOneControls() {}
  private void DriverTwoControls(){}
  private void SingleDriverControls() {
    Command driveFieldOrientatedDirectAngle = swerve.driveFieldOrientated(driveDirectAngle);
    Command driveFieldOrientatedAngularVelocity = swerve.driveFieldOrientated(driveAngularVelocity);

    swerve.setDefaultCommand(driveFieldOrientatedAngularVelocity);

    driverOne.start().whileTrue(new InstantCommand(() -> swerve.zeroGyro()));

    // Elevator
    driverOne.povUp().whileTrue(new cmdElevator_TeleOp(elevator, ()-> 0.2));
    driverOne.povDown().whileTrue(new cmdElevator_TeleOp(elevator, ()-> -0.2));

    // Coral
    driverOne.povLeft().whileTrue(new cmdCoral_TeleOp(coral, ()-> 0.5));
    driverOne.povRight().whileTrue(new cmdCoral_TeleOp(coral, ()-> -0.5));
          
    // Intake
    driverOne.x().whileTrue(new cmdAlgaeIntake_TeleOp(algae, ()-> 0.5));
    driverOne.b().whileTrue(new cmdAlgaeIntake_TeleOp(algae, ()-> -0.5));

    // Arm
    driverOne.a().whileTrue(new cmdAlgaeArm_TeleOp(algae, ()-> 0.2));
    driverOne.y().whileTrue(new cmdAlgaeArm_TeleOp(algae, ()-> -0.2));
  }
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}

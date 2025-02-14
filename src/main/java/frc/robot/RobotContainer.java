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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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

  private void DriverOneControls() {
    /* Command driveFieldOrientatedDirectAngle = swerve.driveFieldOrientated(driveDirectAngle);
    Command driveFieldOrientatedAngularVelocity = swerve.driveFieldOrientated(driveAngularVelocity);

    swerve.setDefaultCommand(driveFieldOrientatedAngularVelocity); */
  }
  private void DriverTwoControls(){}
  private void SingleDriverControls() {
    Command driveFieldOrientatedDirectAngle = swerve.driveFieldOrientated(driveDirectAngle);
    Command driveFieldOrientatedAngularVelocity = swerve.driveFieldOrientated(driveAngularVelocity);

    swerve.setDefaultCommand(driveFieldOrientatedAngularVelocity);

    // Elevator
    driverOne.povUp().whileTrue(new cmdElevator_TeleOp(elevator, ()-> 0.2));
    driverOne.povDown().whileTrue(new cmdElevator_TeleOp(elevator, ()-> -0.2));
    /* elevator.setDefaultCommand(
      new cmdElevator_TeleOp(
          elevator,
          () -> MathUtil.applyDeadband(-driverOne.getLeftY(), 0.05))); */

    // Coral
    driverOne.povLeft().whileTrue(new cmdCoral_TeleOp(coral, ()-> 0.5));
    driverOne.povRight().whileTrue(new cmdCoral_TeleOp(coral, ()-> -0.5));
    /* coral.setDefaultCommand(
      new cmdCoral_TeleOp(
          coral,
          () -> MathUtil.applyDeadband(-driverOne.getRightY(), 0.05))); */
          
    // Intake
    driverOne.x().whileTrue(new cmdAlgaeIntake_TeleOp(algae, ()-> 0.5));
    driverOne.b().whileTrue(new cmdAlgaeIntake_TeleOp(algae, ()-> -0.5));
    driverOne.a().whileTrue(new cmdAlgaeArm_TeleOp(algae, ()-> 0.2));
    driverOne.y().whileTrue(new cmdAlgaeArm_TeleOp(algae, ()-> -0.2));
    //driverOne.getAButton().whileTrue(new cmdAlgaeIntake_TeleOp(algae, ()-> 0.5));
    //driverOne.b().whileTrue(new cmdAlgaeIntake_TeleOp(algae, ()-> -0.5));

    // Arm
    //driverOne.x().whileTrue(new cmdAlgaeArm_TeleOp(algae, ()->0.5));
    //driverOne.y().whileTrue(new cmdAlgaeArm_TeleOp(algae, ()->-0.5));
   
  }
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}

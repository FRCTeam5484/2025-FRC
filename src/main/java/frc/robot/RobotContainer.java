package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.cmdAlgaeArm_TeleOp;
import frc.robot.commands.cmdAlgaeIntake_TeleOp;
import frc.robot.commands.cmdAlgae_AutoEject;
import frc.robot.commands.cmdAlgae_AutoHold;
import frc.robot.commands.cmdAlgae_AutoIntake;
import frc.robot.commands.cmdCoral_AutoIntake;
import frc.robot.commands.cmdCoral_EjectCoral;
import frc.robot.commands.cmdCoral_TeleOp;
import frc.robot.commands.cmdElevator_AutoToPosition;
import frc.robot.commands.cmdElevator_TeleOp;
import frc.robot.commands.cmdElevator_TeleOpNoSafety;
import frc.robot.commands.cmdSwerve_TeleOp;
import frc.robot.subsystems.subSwerve;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.subAlgaeArm;
import frc.robot.subsystems.subAlgaeIntake;
import frc.robot.subsystems.subAlgaeRemover;
import frc.robot.subsystems.subCoral;
import frc.robot.subsystems.subElevator;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
  private final subCoral coral = new subCoral();
  private final subAlgaeIntake algaeIntake = new subAlgaeIntake();
  private final subAlgaeArm algaeArm = new subAlgaeArm();
  private final subAlgaeRemover algaeRemover = new subAlgaeRemover();
  private final subSwerve swerve = new subSwerve();
  private final subElevator elevator = new subElevator();
  private final CommandXboxController driverOne = new CommandXboxController(OperatorConstants.DriverOne);
  private final CommandXboxController driverTwo = new CommandXboxController(OperatorConstants.DriverTwo);
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

  private void DriverOneControls(){
    //Swerve
    swerve.setDefaultCommand(new cmdSwerve_TeleOp(swerve, driveAngularVelocity));
    driverOne.start().whileTrue(new InstantCommand(() -> swerve.zeroGyro()));
          
    // Auto Intake
    driverOne.leftBumper().whileTrue(new cmdAlgae_AutoIntake(algaeArm, algaeIntake));
    driverOne.leftBumper().whileFalse(new cmdAlgae_AutoHold(algaeArm, algaeIntake));
    driverOne.rightBumper().whileTrue(new cmdAlgae_AutoEject(algaeArm, algaeIntake));
    
    // Manual Intake
    driverOne.povRight().whileTrue(new cmdAlgaeIntake_TeleOp(algaeIntake, ()-> 0.6));
    driverOne.povLeft().whileTrue(new cmdAlgaeIntake_TeleOp(algaeIntake, ()-> -0.6));

    // Manual Arm
    driverOne.povUp().whileTrue(new cmdAlgaeArm_TeleOp(algaeArm, ()-> 0.2));
    driverOne.povDown().whileTrue(new cmdAlgaeArm_TeleOp(algaeArm, ()-> -0.2));
  }
  private void DriverTwoControls(){
    // Auto Elevator
    driverTwo.y().whileTrue(new cmdElevator_AutoToPosition(elevator, Constants.Elevator.L4));
    driverTwo.x().whileTrue(new cmdElevator_AutoToPosition(elevator, Constants.Elevator.L3));
    driverTwo.b().whileTrue(new cmdElevator_AutoToPosition(elevator, Constants.Elevator.L2));
    driverTwo.a().whileTrue(new cmdElevator_AutoToPosition(elevator, Constants.Elevator.L1));
    driverTwo.start().whileTrue(new cmdElevator_AutoToPosition(elevator, Constants.Elevator.bottomPosition));
    
    // Manual Elevator
    elevator.setDefaultCommand(new cmdElevator_TeleOp(elevator, driverTwo::getLeftY));  
    driverTwo.povUp().whileTrue(new cmdElevator_TeleOpNoSafety(elevator, () -> 0.6));
    driverTwo.povDown().whileTrue(new cmdElevator_TeleOpNoSafety(elevator, () -> -0.45));


    // Auto Coral
    driverTwo.leftBumper().onTrue(new cmdCoral_AutoIntake(coral));
    driverTwo.rightBumper().onTrue(new cmdCoral_EjectCoral(coral));

    // Manual Coral
    driverTwo.povLeft().whileTrue(new cmdCoral_TeleOp(coral, ()-> 0.5));
    driverTwo.povRight().whileTrue(new cmdCoral_TeleOp(coral, ()-> -0.5));    

    // Manual Algae Remover
    
  }
  private void SingleDriverControls() {
    //Command driveFieldOrientatedDirectAngle = swerve.driveFieldOrientated(driveDirectAngle);

    swerve.setDefaultCommand(new cmdSwerve_TeleOp(swerve, driveAngularVelocity));

    driverOne.start().whileTrue(new InstantCommand(() -> swerve.zeroGyro()));

    // Elevator
    driverOne.povUp().whileTrue(new cmdElevator_TeleOp(elevator, ()-> 0.8));
    driverOne.povDown().whileTrue(new cmdElevator_TeleOp(elevator, ()-> -0.45));

    // Coral
    driverOne.povLeft().whileTrue(new cmdCoral_TeleOp(coral, ()-> 0.5));
    driverOne.povRight().whileTrue(new cmdCoral_TeleOp(coral, ()-> -0.5));
    driverOne.leftBumper().onTrue(new cmdCoral_EjectCoral(coral));
          
    // Intake
    driverOne.x().whileTrue(new cmdAlgaeIntake_TeleOp(algaeIntake, ()-> 0.6));
    driverOne.b().whileTrue(new cmdAlgaeIntake_TeleOp(algaeIntake, ()-> -0.6));

    // Arm
    driverOne.a().whileTrue(new cmdAlgaeArm_TeleOp(algaeArm, ()-> 0.2));
    driverOne.y().whileTrue(new cmdAlgaeArm_TeleOp(algaeArm, ()-> -0.2));
  }
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}

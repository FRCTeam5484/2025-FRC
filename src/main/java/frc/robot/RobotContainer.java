package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.cmdAlgaeArm_TeleOp;
import frc.robot.commands.cmdAlgaeIntake_TeleOp;
import frc.robot.commands.cmdAlgaeRemover_AutoPosition;
import frc.robot.commands.cmdAlgaeRemover_ResetEncoder;
import frc.robot.commands.cmdAlgaeRemover_Stop;
import frc.robot.commands.cmdAlgaeRemover_TeleOp;
import frc.robot.commands.cmdAlgae_AutoEject;
import frc.robot.commands.cmdAlgae_AutoHold;
import frc.robot.commands.cmdAlgae_AutoIntake;
import frc.robot.commands.cmdCoral_AutoIntake;
import frc.robot.commands.cmdCoral_EjectCoral;
import frc.robot.commands.cmdCoral_Stop;
import frc.robot.commands.cmdCoral_TeleOp;
import frc.robot.commands.cmdElevator_AutoToPosition;
import frc.robot.commands.cmdElevator_Stop;
import frc.robot.commands.cmdElevator_TeleOp;
import frc.robot.commands.cmdElevator_TeleOpNoSafety;
import frc.robot.commands.cmdStopAllCommands;
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
  private final CommandJoystick buttonBoxControllerOne = new CommandJoystick(OperatorConstants.ButtonBoxControllerOne);
  private final CommandJoystick buttonBoxControllerTwo = new CommandJoystick(OperatorConstants.ButtonBoxControllerTwo);
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
    //ButtonBoxControls();
    //TestControls();
  }

  private void TestControls() {
    driverOne.back().whileTrue(new cmdStopAllCommands(algaeArm, algaeIntake, algaeRemover, coral, elevator));
    driverOne.start().whileTrue(new InstantCommand(()->algaeArm.reset()));
    driverOne.leftBumper().whileTrue(new cmdAlgae_AutoIntake(algaeArm, algaeIntake));
    driverOne.leftBumper().whileFalse(new cmdAlgae_AutoHold(algaeArm, algaeIntake));
    driverOne.rightBumper().whileTrue(new cmdAlgae_AutoEject(algaeArm, algaeIntake));
    
    driverOne.povUp().whileTrue(new cmdAlgaeArm_TeleOp(algaeArm, ()-> 0.2));
    driverOne.povDown().whileTrue(new cmdAlgaeArm_TeleOp(algaeArm, ()-> -0.2));
    driverOne.povRight().whileTrue(new cmdAlgaeIntake_TeleOp(algaeIntake, ()-> 0.6));
    driverOne.povLeft().whileTrue(new cmdAlgaeIntake_TeleOp(algaeIntake, ()-> -0.6));
  }

  private void DriverOneControls(){
    //Swerve
    Command driveFieldOrientatedDirectAngle = swerve.driveFieldOrientated(driveDirectAngle);
    Command driveFieldOrientatedAngularVelocity = swerve.driveFieldOrientated(driveAngularVelocity);

    swerve.setDefaultCommand(driveFieldOrientatedAngularVelocity);
    driverOne.start().whileTrue(new InstantCommand(() -> swerve.zeroGyro()));
          
    // Auto Intake
    driverOne.leftBumper().whileTrue(new cmdAlgae_AutoIntake(algaeArm, algaeIntake));
    driverOne.leftBumper().whileFalse(new cmdAlgae_AutoHold(algaeArm, algaeIntake));
    driverOne.rightBumper().whileTrue(new cmdAlgae_AutoEject(algaeArm, algaeIntake));
    
    // Manual Intake
    driverOne.leftTrigger().whileTrue(new cmdAlgaeIntake_TeleOp(algaeIntake, ()-> driverOne.getLeftTriggerAxis()));
    driverOne.rightTrigger().whileTrue(new cmdAlgaeIntake_TeleOp(algaeIntake, ()-> driverOne.getRightTriggerAxis()));
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
    
    // Manual Elevator
    elevator.setDefaultCommand(new cmdElevator_TeleOp(elevator, driverTwo::getLeftY));  
    driverTwo.povUp().whileTrue(new cmdElevator_TeleOpNoSafety(elevator, () -> 0.5));
    driverTwo.povDown().whileTrue(new cmdElevator_TeleOpNoSafety(elevator, () -> -0.45));

    // Auto Coral
    driverTwo.leftBumper().onTrue(new cmdCoral_AutoIntake(coral));
    driverTwo.rightBumper().onTrue(new cmdCoral_EjectCoral(coral));

    // Manual Coral
    driverTwo.povLeft().whileTrue(new cmdCoral_TeleOp(coral, ()-> 0.5));
    driverTwo.povRight().whileTrue(new cmdCoral_TeleOp(coral, ()-> -0.5));    

    // Manual Algae Remover
    algaeRemover.setDefaultCommand(new cmdAlgaeRemover_TeleOp(algaeRemover, driverTwo::getRightY));
    driverTwo.back().whileTrue(new cmdAlgaeRemover_TeleOp(algaeRemover, ()->0.2));
    driverTwo.start().whileTrue(new cmdAlgaeRemover_TeleOp(algaeRemover, ()->-0.2));
  }
  private void SingleDriverControls() {
    Command driveFieldOrientatedDirectAngle = swerve.driveFieldOrientated(driveDirectAngle);
    Command driveFieldOrientatedAngularVelocity = swerve.driveFieldOrientated(driveAngularVelocity);

    swerve.setDefaultCommand(driveFieldOrientatedAngularVelocity);
    driverOne.start().whileTrue(new InstantCommand(() -> swerve.zeroGyro()));

    // Elevator
    driverOne.povUp().whileTrue(new cmdElevator_TeleOp(elevator, ()-> 1));
    driverOne.povDown().whileTrue(new cmdElevator_TeleOp(elevator, ()-> -0.7));
    driverOne.y().whileTrue(new cmdElevator_AutoToPosition(elevator, Constants.Elevator.L4));
    driverOne.x().whileTrue(new cmdElevator_AutoToPosition(elevator, Constants.Elevator.L3));
    driverOne.b().whileTrue(new cmdElevator_AutoToPosition(elevator, Constants.Elevator.L2));
    driverOne.a().whileTrue(new cmdElevator_AutoToPosition(elevator, Constants.Elevator.L1));

    // Coral
    driverOne.leftBumper().onTrue(new cmdCoral_AutoIntake(coral));
    driverOne.rightBumper().onTrue(new cmdCoral_EjectCoral(coral));

    // Algae Remover
    driverOne.leftTrigger().whileTrue(new cmdAlgaeRemover_AutoPosition(algaeRemover, Constants.Algae.RemoverArmDown));
    driverOne.rightTrigger().whileTrue(new cmdAlgaeRemover_AutoPosition(algaeRemover, Constants.Algae.RemoverArmUp));
  }
  private void ButtonBoxControls(){
    buttonBoxControllerOne.button(1).whileTrue(new cmdElevator_AutoToPosition(elevator, Constants.Elevator.L4));
    buttonBoxControllerOne.button(2).whileTrue(new cmdElevator_AutoToPosition(elevator, Constants.Elevator.L3));
    buttonBoxControllerOne.button(3).whileTrue(new cmdElevator_AutoToPosition(elevator, Constants.Elevator.L2));
    buttonBoxControllerOne.button(4).whileTrue(new cmdElevator_AutoToPosition(elevator, Constants.Elevator.L1));
    buttonBoxControllerOne.button(5).whileTrue(new cmdElevator_AutoToPosition(elevator, Constants.Elevator.bottomPosition));
    buttonBoxControllerOne.button(6).whileTrue(new cmdElevator_Stop(elevator));
    buttonBoxControllerOne.button(7).whileTrue(new cmdCoral_TeleOp(coral, ()->-0.5));
    buttonBoxControllerOne.button(8).whileTrue(new cmdCoral_TeleOp(coral, ()->0.5));
    buttonBoxControllerOne.button(9).whileTrue(new cmdCoral_EjectCoral(coral));
    buttonBoxControllerOne.button(10).whileTrue(new cmdCoral_AutoIntake(coral));

    buttonBoxControllerTwo.button(1).whileTrue(new cmdCoral_Stop(coral));
    buttonBoxControllerTwo.button(2).whileTrue(new cmdAlgaeRemover_AutoPosition(algaeRemover, Constants.Algae.RemoverArmUp));
    buttonBoxControllerTwo.button(3).whileTrue(new cmdAlgaeRemover_AutoPosition(algaeRemover, Constants.Algae.RemoverArmDown));
    buttonBoxControllerTwo.button(4).whileTrue(new cmdAlgaeRemover_AutoPosition(algaeRemover, Constants.Algae.RemoverArmUp));
    buttonBoxControllerTwo.button(5).whileTrue(new cmdAlgaeRemover_Stop(algaeRemover));
    //buttonBoxControllerTwo.button(6).whileTrue(new cmdCoral_TeleOp(coral, ()->-0.5));
    //buttonBoxControllerTwo.button(7).whileTrue(new cmdCoral_TeleOp(coral, ()->0.5));
    //buttonBoxControllerTwo.button(8).whileTrue(new cmdCoral_EjectCoral(coral));
    //buttonBoxControllerTwo.button(9).whileTrue(new cmdCoral_AutoIntake(coral));
    //buttonBoxControllerTwo.button(10).whileTrue(new cmdCoral_Stop(coral));
  }
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}

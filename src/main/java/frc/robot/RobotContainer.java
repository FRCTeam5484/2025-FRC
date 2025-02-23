package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.cmdAuto_AlgaeRemoverToPosition;
import frc.robot.commands.cmdAlgaeRemover_ResetEncoder;
import frc.robot.commands.cmdAlgaeRemover_Stop;
import frc.robot.commands.cmdAlgaeRemover_TeleOp;
import frc.robot.commands.cmdAuto_AlgaeEject;
import frc.robot.commands.cmdAuto_AlgaeHold;
import frc.robot.commands.cmdAuto_AlgaeIntake;
import frc.robot.commands.cmdAuto_CoralIntake;
import frc.robot.commands.cmdAuto_CoralEject;
import frc.robot.commands.cmdCoral_Stop;
import frc.robot.commands.cmdCoral_TeleOp;
import frc.robot.commands.cmdAuto_EvevatorToPosition;
import frc.robot.commands.cmdElevator_Stop;
import frc.robot.commands.cmdElevator_TeleOp;
import frc.robot.subsystems.subSwerve;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.subAlgaeProcessor;
import frc.robot.subsystems.subAlgaeRemover;
import frc.robot.subsystems.subBlinkin;
import frc.robot.subsystems.subCoral;
import frc.robot.subsystems.subElevator;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
  public final subCoral coral = new subCoral();
  public final subAlgaeProcessor algaeProcessor = new subAlgaeProcessor();
  public final subAlgaeRemover algaeRemover = new subAlgaeRemover();
  public final subSwerve swerve = new subSwerve();
  public final subElevator elevator = new subElevator();
  public final subBlinkin blinkin = new subBlinkin();
  private final CommandXboxController driverOne = new CommandXboxController(OperatorConstants.DriverOne);
  private final CommandJoystick buttonBoxControllerOne = new CommandJoystick(OperatorConstants.ButtonBoxControllerOne);
  private final CommandJoystick buttonBoxControllerTwo = new CommandJoystick(OperatorConstants.ButtonBoxControllerTwo);
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(), 
                                                                () -> driverOne.getLeftY() * -1, 
                                                                () -> driverOne.getLeftX() * -1)
                                                                .withControllerRotationAxis(driverOne::getRightX)
                                                                .deadband(0.01)
                                                                .scaleTranslation(0.5)
                                                                .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverOne::getRightX,
                                                                                            driverOne::getRightY)
                                                                                           .headingWhile(true);
  
  public RobotContainer() {
    DriverOneControls();
    ButtonBoxControls();
  }

  private void DriverOneControls(){
    //Swerve
    Command driveFieldOrientatedDirectAngle = swerve.driveFieldOrientated(driveDirectAngle);
    Command driveFieldOrientatedAngularVelocity = swerve.driveFieldOrientated(driveAngularVelocity);

    swerve.setDefaultCommand(driveFieldOrientatedAngularVelocity);
    driverOne.start().whileTrue(new InstantCommand(() -> swerve.zeroGyro()));
          
    // Auto Intake
    driverOne.leftBumper().whileTrue(new cmdAuto_AlgaeIntake(algaeProcessor));
    driverOne.leftBumper().whileFalse(new cmdAuto_AlgaeHold(algaeProcessor));
    driverOne.rightBumper().whileTrue(new cmdAuto_AlgaeEject(algaeProcessor));
  }

  private void ButtonBoxControls(){
    buttonBoxControllerOne.button(1).onTrue(new cmdAuto_EvevatorToPosition(elevator, blinkin, Constants.Elevator.L4));
    buttonBoxControllerOne.button(2).onTrue(new cmdAuto_EvevatorToPosition(elevator, blinkin, Constants.Elevator.L3));
    buttonBoxControllerOne.button(3).onTrue(new cmdAuto_EvevatorToPosition(elevator, blinkin, Constants.Elevator.L2));
    buttonBoxControllerOne.button(4).onTrue(new cmdAuto_EvevatorToPosition(elevator, blinkin, Constants.Elevator.L1));
    buttonBoxControllerOne.button(5).onTrue(new cmdAuto_EvevatorToPosition(elevator, blinkin, Constants.Elevator.bottomPosition));
    buttonBoxControllerOne.button(6).onTrue(new cmdElevator_Stop(elevator));
    buttonBoxControllerOne.button(7).whileTrue(new cmdCoral_TeleOp(coral, ()->-0.5));
    buttonBoxControllerOne.button(8).whileTrue(new cmdCoral_TeleOp(coral, ()->0.5));
    buttonBoxControllerOne.button(9).onTrue(new cmdAuto_CoralEject(coral, blinkin));
    buttonBoxControllerOne.button(10).onTrue(new cmdAuto_CoralIntake(coral, blinkin));
    buttonBoxControllerOne.button(11).onTrue(new cmdCoral_Stop(coral));
    elevator.setDefaultCommand(new cmdElevator_TeleOp(elevator, ()->buttonBoxControllerOne.getRawAxis(1)*.5));


    buttonBoxControllerTwo.button(1).onTrue(new cmdAuto_AlgaeRemoverToPosition(algaeRemover, Constants.Algae.RemoverArmDown));
    buttonBoxControllerTwo.button(2).onTrue(new cmdAuto_AlgaeRemoverToPosition(algaeRemover, Constants.Algae.RemoverArmUp));
    buttonBoxControllerTwo.button(3).whileTrue(new cmdAlgaeRemover_TeleOp(algaeRemover, ()->-0.25));
    buttonBoxControllerTwo.button(4).whileTrue(new cmdAlgaeRemover_TeleOp(algaeRemover, ()->0.15));
    buttonBoxControllerTwo.button(5).onTrue(new cmdAlgaeRemover_Stop(algaeRemover));
    buttonBoxControllerTwo.button(6).onTrue(new cmdAlgaeRemover_ResetEncoder(algaeRemover));
    algaeRemover.setDefaultCommand(new cmdAlgaeRemover_TeleOp(algaeRemover, ()-> -buttonBoxControllerTwo.getRawAxis(1)*0.25));
  }
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
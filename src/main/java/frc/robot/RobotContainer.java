package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.cmdAuto_AlgaeRemoverToPosition;
import frc.robot.commands.cmdAlgaeProcessor_TeleOp;
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
import frc.robot.commands.cmdSwerve_TeleOp;
import frc.robot.subsystems.subSwerve;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.subAlgaeProcessor;
import frc.robot.subsystems.subAlgaeRemover;
import frc.robot.subsystems.subBlinkin;
import frc.robot.subsystems.subCoral;
import frc.robot.subsystems.subElevator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
  private SendableChooser<Command> chooser = new SendableChooser<>();
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
                                                                .scaleTranslation(0.65)
                                                                .allianceRelativeControl(false);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverOne::getRightX,
                                                                                            driverOne::getRightY)
                                                                                           .headingWhile(true);
  
  public RobotContainer() {
    DriverOneControls();
    ButtonBoxControls();

    // Named Commands
    NamedCommands.registerCommand("Coral Intake", new cmdAuto_CoralIntake(coral, blinkin));
    NamedCommands.registerCommand("Coral Eject", new cmdAuto_CoralEject(coral, blinkin));
    NamedCommands.registerCommand("Elevator Bottom", new cmdAuto_EvevatorToPosition(elevator, blinkin, Constants.Elevator.bottomPosition));
    NamedCommands.registerCommand("Elevator L1", new cmdAuto_EvevatorToPosition(elevator, blinkin, Constants.Elevator.L1));
    NamedCommands.registerCommand("Elevator L2", new cmdAuto_EvevatorToPosition(elevator, blinkin, Constants.Elevator.L2));
    NamedCommands.registerCommand("Elevator L3", new cmdAuto_EvevatorToPosition(elevator, blinkin, Constants.Elevator.L3));
    NamedCommands.registerCommand("Elevator L4", new cmdAuto_EvevatorToPosition(elevator, blinkin, Constants.Elevator.L4));

    //addAutoOptions();
  }

  private void DriverOneControls(){
    //Swerve
    //swerve.setDefaultCommand(swerve.driveFieldOrientated(driveDirectAngle));
    swerve.setDefaultCommand(swerve.driveFieldOrientated(driveAngularVelocity));
    //swerve.setDefaultCommand(new cmdSwerve_TeleOp(swerve, ()->driverOne.getLeftY(), ()->driverOne.getLeftX(), ()->driverOne.getRightX(), ()->driverOne.getRightY()));
    driverOne.start().whileTrue(new InstantCommand(() -> swerve.zeroGyro()));
          
    // Auto Intake
    driverOne.leftBumper().whileTrue(new cmdAuto_AlgaeIntake(algaeProcessor));
    driverOne.leftBumper().whileFalse(new cmdAuto_AlgaeHold(algaeProcessor));
    driverOne.rightBumper().whileTrue(new cmdAuto_AlgaeEject(algaeProcessor));

    driverOne.povUp().whileTrue(new cmdAlgaeProcessor_TeleOp(algaeProcessor, ()->0.3));
    driverOne.povDown().whileTrue(new cmdAlgaeProcessor_TeleOp(algaeProcessor, ()->-0.3));
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
  private void addAutoOptions(){
    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Options", chooser);
  }
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
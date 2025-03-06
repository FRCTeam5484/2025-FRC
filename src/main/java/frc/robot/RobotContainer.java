package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.cmdAuto_AlgaeRemoverToPosition;
import frc.robot.commands.cmdAuto_AlignRobot;
import frc.robot.commands.cmdAlgaeRemover_ResetEncoder;
import frc.robot.commands.cmdAlgaeRemover_Stop;
import frc.robot.commands.cmdAlgaeRemover_TeleOp;
import frc.robot.commands.cmdAuto_CoralIntake;
import frc.robot.commands.cmdAuto_CoralEject;
import frc.robot.commands.cmdCoral_Stop;
import frc.robot.commands.cmdCoral_TeleOp;
import frc.robot.commands.cmdAuto_EvevatorToPosition;
import frc.robot.commands.cmdElevator_Stop;
import frc.robot.commands.cmdElevator_TeleOp;
import frc.robot.subsystems.subSwerve;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.subAlgaeRemover;
import frc.robot.subsystems.subBlinkin;
import frc.robot.subsystems.subCoral;
import frc.robot.subsystems.subElevator;
import frc.robot.subsystems.subLimelight;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
  private SendableChooser<Command> chooser = new SendableChooser<>();
  public final subCoral coral = new subCoral();
  //public final subAlgaeProcessor algaeProcessor = new subAlgaeProcessor();
  public final subAlgaeRemover algaeRemover = new subAlgaeRemover();
  public final subSwerve swerve  = new subSwerve(new File(Filesystem.getDeployDirectory(), "swerve"));
  public final subElevator elevator = new subElevator();
  //public final subBlinkin blinkin = new subBlinkin();
  public final subLimelight limelight = new subLimelight();
  private final CommandXboxController driverOne = new CommandXboxController(OperatorConstants.DriverOne);
  private final CommandJoystick buttonBoxControllerOne = new CommandJoystick(OperatorConstants.ButtonBoxControllerOne);
  private final CommandJoystick buttonBoxControllerTwo = new CommandJoystick(OperatorConstants.ButtonBoxControllerTwo);
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(),
                                                                () -> driverOne.getLeftY() * -1,
                                                                () -> driverOne.getLeftX() * -1)
                                                            .withControllerRotationAxis(()->driverOne.getRightX()*-1)
                                                            .deadband(0.05)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);
  
  public RobotContainer() {
    DriverOneControls();
    ButtonBoxControls();

    // Named Commands
    NamedCommands.registerCommand("Coral Intake", new cmdAuto_CoralIntake(coral));
    NamedCommands.registerCommand("Coral Eject", new cmdAuto_CoralEject(coral).withTimeout(2));
    NamedCommands.registerCommand("Limelight Lineup", new cmdAuto_AlignRobot(swerve, limelight).withTimeout(5));
    NamedCommands.registerCommand("Algae Remover Up", new cmdAuto_AlgaeRemoverToPosition(algaeRemover, Constants.Algae.RemoverArmUp).withTimeout(1));
    NamedCommands.registerCommand("Algae Remover Down", new cmdAuto_AlgaeRemoverToPosition(algaeRemover, Constants.Algae.RemoverArmDown).withTimeout(1));
    NamedCommands.registerCommand("Elevator Bottom", new cmdAuto_EvevatorToPosition(elevator, Constants.Elevator.bottomPosition));
    NamedCommands.registerCommand("Elevator L1", new cmdAuto_EvevatorToPosition(elevator, Constants.Elevator.L1).withTimeout(2));
    NamedCommands.registerCommand("Elevator L2", new cmdAuto_EvevatorToPosition(elevator, Constants.Elevator.L2).withTimeout(2));
    NamedCommands.registerCommand("Elevator L3", new cmdAuto_EvevatorToPosition(elevator, Constants.Elevator.L3).withTimeout(2));
    NamedCommands.registerCommand("Elevator L4", new cmdAuto_EvevatorToPosition(elevator, Constants.Elevator.L4).withTimeout(2));

    addAutoOptions();
  }

  private void DriverOneControls(){
    //Swerve
    //Command driveFieldOrientedDirectAngle      = swerve.driveFieldOriented(driveDirectAngle);
    //Command driveFieldOrientedAnglularVelocity = swerve.driveFieldOriented(driveAngularVelocity);
    //Command driveRobotOrientedAngularVelocity  = swerve.driveFieldOriented(driveRobotOriented);
    //Command driveSetpointGen = swerve.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    //swerve.setDefaultCommand(swerve.driveFieldOriented(driveAngularVelocity));
    swerve.setDefaultCommand(
        swerve.driveFieldOriented(
            driveRobotOriented.robotRelative(()->driverOne.getRightTriggerAxis()>0.5?true:false)
            .allianceRelativeControl(()->driverOne.getRightTriggerAxis()>0.5?false:true)));
    //swerve.setDefaultCommand(new cmdSwerve_TeleOp(swerve, ()->driverOne.getLeftY()*-1, ()->driverOne.getLeftX()*-1, ()->driverOne.getRightX()*-1, ()->driverOne.getRightTriggerAxis()>0.5 ? false : true));
    //swerve.setDefaultCommand(new cmdSwerve_TeleOpAlt(swerve, ()->driverOne.getLeftY()*-1, ()->driverOne.getLeftX()*-1, ()->driverOne.getRightX()*-1, ()->driverOne.getRightTriggerAxis()>0.5 ? false : true));
    
    if (DriverStation.isTest())
    {
      driverOne.start().onTrue((Commands.runOnce(swerve::zeroGyro)));
      driverOne.back().whileTrue(swerve.centerModulesCommand());

      driverOne.a().whileTrue(Commands.none());
      driverOne.b().whileTrue(Commands.none());
      driverOne.x().whileTrue(Commands.runOnce(swerve::lock, swerve).repeatedly());
      driverOne.y().whileTrue(swerve.driveToDistanceCommand(1.0, 0.2));

      driverOne.leftBumper().onTrue(Commands.none());
      driverOne.rightBumper().onTrue(Commands.none());

      driverOne.povUp().onTrue(Commands.none());
      driverOne.povDown().onTrue(Commands.none());
      driverOne.povLeft().onTrue(Commands.none());
      driverOne.povRight().onTrue(Commands.none());
    } 
    else
    {
      driverOne.start().whileTrue(new InstantCommand(() -> swerve.zeroGyro()));
      driverOne.back().onTrue(Commands.runOnce(()->swerve.resetOdometry(new Pose2d(3,3, new Rotation2d()))));      

      driverOne.a().whileTrue(new cmdAuto_AlignRobot(swerve, limelight));
      driverOne.b().onTrue(Commands.none());
      driverOne.x().onTrue(Commands.none());
      driverOne.y().onTrue(Commands.none());
            
      // Auto Intake
      //driverOne.leftBumper().whileTrue(new cmdAuto_AlgaeIntake(algaeProcessor));
      //driverOne.leftBumper().whileFalse(new cmdAuto_AlgaeHold(algaeProcessor));
      //driverOne.rightBumper().whileTrue(new cmdAuto_AlgaeEject(algaeProcessor));

      driverOne.povUp().whileTrue(new cmdElevator_TeleOp(elevator, ()->0.3));
      driverOne.povDown().whileTrue(new cmdElevator_TeleOp(elevator, ()->-0.3));
      driverOne.povLeft().onTrue(Commands.none());
      driverOne.povRight().onTrue(Commands.none());
    }
  }

  private void ButtonBoxControls(){
    //algaeRemover.setDefaultCommand(new cmdAuto_AlgaeRemoverToPosition(algaeRemover, Constants.Algae.RemoverArmUp));

    buttonBoxControllerOne.button(1).onTrue(new cmdAuto_EvevatorToPosition(elevator, Constants.Elevator.L4));
    buttonBoxControllerOne.button(2).onTrue(new cmdAuto_EvevatorToPosition(elevator, Constants.Elevator.L3));
    buttonBoxControllerOne.button(3).onTrue(new cmdAuto_EvevatorToPosition(elevator, Constants.Elevator.L2));
    buttonBoxControllerOne.button(4).onTrue(new cmdAuto_EvevatorToPosition(elevator, Constants.Elevator.L1));
    buttonBoxControllerOne.button(5).onTrue(new cmdAuto_EvevatorToPosition(elevator, Constants.Elevator.bottomPosition));
    buttonBoxControllerOne.button(6).onTrue(new cmdElevator_Stop(elevator));
    buttonBoxControllerOne.button(7).whileTrue(new cmdCoral_TeleOp(coral, ()->-0.5));
    buttonBoxControllerOne.button(8).whileTrue(new cmdCoral_TeleOp(coral, ()->0.5));
    buttonBoxControllerOne.button(9).onTrue(new cmdAuto_CoralEject(coral));
    buttonBoxControllerOne.button(10).onTrue(new cmdAuto_CoralIntake(coral));
    buttonBoxControllerOne.button(11).onTrue(new cmdCoral_Stop(coral));
    elevator.setDefaultCommand(new cmdElevator_TeleOp(elevator, ()->-buttonBoxControllerOne.getRawAxis(1)*.5));


    buttonBoxControllerTwo.button(1).onTrue(new cmdAuto_AlgaeRemoverToPosition(algaeRemover, Constants.Algae.RemoverArmDown));
    buttonBoxControllerTwo.button(2).onTrue(new cmdAuto_AlgaeRemoverToPosition(algaeRemover, Constants.Algae.RemoverArmUp));
    buttonBoxControllerTwo.button(3).whileTrue(new cmdAlgaeRemover_TeleOp(algaeRemover, ()->-0.25));
    buttonBoxControllerTwo.button(4).whileTrue(new cmdAlgaeRemover_TeleOp(algaeRemover, ()->0.15));
    buttonBoxControllerTwo.button(5).onTrue(new cmdAlgaeRemover_Stop(algaeRemover));
    buttonBoxControllerTwo.button(6).onTrue(new cmdAlgaeRemover_ResetEncoder(algaeRemover));
    algaeRemover.setDefaultCommand(new cmdAlgaeRemover_TeleOp(algaeRemover, ()-> -buttonBoxControllerTwo.getRawAxis(1)*0.25));
  }
  private void addAutoOptions(){
    chooser.addOption("Crossline Only", swerve.getAutonomousCommand("Crossline Only"));
    chooser.addOption("Place 1 Coral", swerve.getAutonomousCommand("Place 1 Coral"));
    chooser.addOption("2 Coral - Center to Right Human Feed", swerve.getAutonomousCommand("2 Coral - Center to Right Human Feed"));
    //chooser.addOption("Drive Forward and Wait", swerve.getAutonomousCommand("Drive Forward and Wait"));
    SmartDashboard.putData("Autonomous Options", chooser);
  }
  public Command getAutonomousCommand() {
    //return new cmdAutonomous_Crossline(swerve); 
    return chooser.getSelected();
  }
}
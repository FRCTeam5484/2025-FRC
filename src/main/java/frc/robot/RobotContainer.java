// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.cmdAlgaeArm_TeleOp;
import frc.robot.commands.cmdAlgaeIntake_TeleOp;
import frc.robot.commands.cmdCoral_TeleOp;
import frc.robot.commands.cmdElevator_TeleOp;
import frc.robot.subsystems.subAlgae;
import frc.robot.subsystems.subCoral;
import frc.robot.subsystems.subElevator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  //private final subCoral coral = new subCoral();
  //private final subAlgae algae = new subAlgae();
  private final subElevator elevator = new subElevator();

  private final CommandXboxController driverOne = new CommandXboxController(OperatorConstants.DriverOne);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    configureDriverOne();
    configureDriverTwo();
    configureSingleDriver();
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }
  private void configureDriverOne() {}
  private void configureDriverTwo(){}
  private void configureSingleDriver() {
    // Elevator
    elevator.setDefaultCommand(
      new cmdElevator_TeleOp(
          elevator,
          () -> MathUtil.applyDeadband(-driverOne.getLeftY(), 0.05)));

    // Coral
    /*
    coral.setDefaultCommand(
      new cmdCoral_TeleOp(
          coral,
          () -> MathUtil.applyDeadband(-driverOne.getRightY(), 0.05)));
          
    // Intake
    driverOne.a().whileTrue(new cmdAlgaeIntake_TeleOp(algae, ()-> 0.5));
    driverOne.b().whileTrue(new cmdAlgaeIntake_TeleOp(algae, ()-> -0.5));

    // Arm
    driverOne.x().whileTrue(new cmdAlgaeArm_TeleOp(algae, ()->0.5));
    driverOne.y().whileTrue(new cmdAlgaeArm_TeleOp(algae, ()->-0.5));
    */
  }
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}

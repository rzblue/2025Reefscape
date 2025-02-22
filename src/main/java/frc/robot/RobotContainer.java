package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.*;
import monologue.Logged;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Logged {
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
      new JoystickButton(driver.getHID(), XboxController.Button.kY.value);

  /* Subsystems */
  private final Swerve swerve = new Swerve();
  private final Elevator elevator = new Elevator();
  private final CoralHead coralHead = new CoralHead();
  private final AlgaeKicker algaeKicker = new AlgaeKicker();
  private final Climber climber = new Climber();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve.setDefaultCommand(
        swerve.teleopDriveCommand(
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis)));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroHeading()));

    operator.b().onTrue(elevator.positionCommand(0)); // Stow
    operator.a().onTrue(elevator.positionCommand(9.5)); // L2
    operator.x().onTrue(elevator.positionCommand(25.5)); // L3
    operator.y().onTrue(elevator.positionCommand(52)); // L4(?)/max

    operator
        .axisLessThan(XboxController.Axis.kLeftY.value, -0.2)
        .onTrue(elevator.relativePositionCommand(1));
    operator
        .axisGreaterThan(XboxController.Axis.kLeftY.value, 0.2)
        .onTrue(elevator.relativePositionCommand(-1));

    // operator
    // .leftTrigger(0.01)
    // .or(operator.rightTrigger(0.01))
    // .whileTrue(
    //     coralHead.outputCommand(
    //         () -> (operator.getRightTriggerAxis() - operator.getLeftTriggerAxis())));
    operator
        .rightTrigger(.1)
        .whileTrue(elevator.positionCommand(0).alongWith(coralHead.outputCommand(() -> 0.075)));

    operator.leftTrigger(.1).whileTrue(coralHead.outputCommand(() -> .25));
    operator.leftBumper().or(operator.rightBumper()).whileTrue(coralHead.outputCommand(() -> -0.15));


    operator.povDown().whileTrue(algaeKicker.outputCommand(() -> 0.5));
    operator.povUp().whileTrue(algaeKicker.outputCommand(() -> -0.5));
    operator.back().onTrue(climber.runOnce(() -> climber.unlatch()));

    operator.start().onTrue(climber.runOnce(() -> climber.latch()));

    RobotModeTriggers.disabled()
        .onTrue(climber.runOnce(() -> climber.latch()).ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}

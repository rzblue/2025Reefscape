package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

  /* Driver Buttons */
  private final Trigger robotCentric = driver.leftBumper();

  /* Subsystems */
  private final Swerve swerve = new Swerve();
  private final Elevator elevator = new Elevator();
  private final CoralHead coralHead = new CoralHead();
  private final AlgaeKicker algaeKicker = new AlgaeKicker();
  private final Climber climber = new Climber();

  private final SendableChooser<Command> autoChooser;

  private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve.setDefaultCommand(
        swerve.teleopDriveCommand(
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            robotCentric));

    // Configure the button bindings
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser();

    setupDashboard();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */

    // Zero heading
    driver.y().onTrue(new InstantCommand(() -> swerve.zeroHeading()));

    operator.b().onTrue(elevator.positionCommand(0)); // Stow
    operator.a().onTrue(elevator.positionCommand(9.5)); // L2
    operator.x().onTrue(elevator.positionCommand(25.5)); // L3
    operator.y().onTrue(elevator.positionCommand(50)); // L4(?)/max

    // Bump down
    operator
        .axisLessThan(XboxController.Axis.kLeftY.value, -0.2)
        .onTrue(elevator.relativePositionCommand(1));
    // Bump Up
    operator
        .axisGreaterThan(XboxController.Axis.kLeftY.value, 0.2)
        .onTrue(elevator.relativePositionCommand(-1));

    // Intake
    operator
        .rightTrigger(.1)
        // .whileTrue(elevator.positionCommand(0).alongWith(coralHead.operatorIntake()));
        .whileTrue(
            elevator
                .positionCommand(0)
                .alongWith(coralHead.smartIntake())
                .alongWith(
                    Commands.waitUntil(coralHead::coralAquired)
                        .andThen(
                            Commands.runOnce(() -> operator.setRumble(RumbleType.kBothRumble, 1))))
                .finallyDo(() -> operator.setRumble(RumbleType.kBothRumble, 0)));

    // Extend
    operator.rightBumper().whileTrue(coralHead.smartExtend());

    // Score
    operator.leftTrigger(.1).whileTrue(coralHead.operatorScore());

    // Retract
    operator.leftBumper().whileTrue(coralHead.operatorRetract());

    // Kick algae
    operator.povUp().whileTrue(algaeKicker.kickAlgae());

    // Climb
    operator.back().onTrue(climber.runOnce(() -> climber.deploy()));
    // Reset climber
    operator.start().onTrue(climber.runOnce(() -> climber.reset()));

    RobotModeTriggers.disabled()
        .onTrue(climber.runOnce(() -> climber.reset()).ignoringDisable(true));
  }

  public void setupDashboard() {
    driverTab.add(autoChooser).withPosition(0, 0).withSize(2, 1);
    Shuffleboard.selectTab("Driver");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

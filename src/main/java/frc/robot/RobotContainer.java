package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.CommandUtils;
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
  private final Trigger slowMode = driver.rightBumper();

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
            robotCentric,
            slowMode));

    NamedCommands.registerCommand("shootCoral", coralHead.smartScore());
    NamedCommands.registerCommand("intakeCoral", coralHead.smartIntake());
    NamedCommands.registerCommand("elevatorStow", elevator.stowCommand());
    NamedCommands.registerCommand("elevatorL2", elevator.l2Command());
    NamedCommands.registerCommand("elevatorL3", elevator.l3Command());
    NamedCommands.registerCommand("elevatorL4", elevator.l4Command());
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

    operator.b().onTrue(elevator.stowCommand()); // Stow
    operator.a().onTrue(elevator.l2Command()); // L2
    operator.x().onTrue(elevator.l3Command()); // L3
    operator.y().onTrue(elevator.l4Command()); // L4(?)/max

    operator.back().debounce(0.125).onTrue(elevator.currentHome());

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
        .whileTrue(elevator.positionCommand(0).alongWith(coralHead.smartIntake()));

    operator
        .rightTrigger(0.1)
        .and(coralHead::coralAquired)
        .whileTrue(
            CommandUtils.rumbleController(operator, 1)
                .alongWith(CommandUtils.rumbleController(driver, 1)));

    // Extend
    operator.rightBumper().whileTrue(coralHead.smartExtend());

    // Score
    operator.leftTrigger(.1).whileTrue(coralHead.operatorScore());

    // Retract
    operator.leftBumper().whileTrue(coralHead.operatorRetract());

    // Kick algae
    operator.povUp().whileTrue(algaeKicker.kickAlgae());

    // Climb
    driver.a().and(driver.x()).onTrue(climber.runOnce(() -> climber.deploy()));
    // Reset climber
    driver.start().onTrue(climber.runOnce(() -> climber.reset()));

    RobotModeTriggers.disabled()
        .onTrue(climber.runOnce(() -> climber.reset()).ignoringDisable(true));

    RobotModeTriggers.autonomous().onTrue(algaeKicker.extend());
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

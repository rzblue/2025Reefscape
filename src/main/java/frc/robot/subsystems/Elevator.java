package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

public class Elevator extends SubsystemBase implements Logged {
  private final TalonFX leader = new TalonFX(leaderId);
  private final TalonFX follower = new TalonFX(followerId);

  private final Follower followReq = new Follower(leader.getDeviceID(), false);
  private final MotionMagicVoltage posRequest = new MotionMagicVoltage(0);

  private final Canandmag encoder = new Canandmag(1);

  private boolean initialHomeComplete = false;

  public Elevator() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    config.CurrentLimits.withStatorCurrentLimit(60)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40)
        .withSupplyCurrentLowerLimit(40)
        .withSupplyCurrentLimitEnable(true);

    follower.getConfigurator().apply(config);

    config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(52.4)
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(-10)
        .withReverseSoftLimitEnable(true);

    config.Slot0.withKP(3);
    config.MotionMagic.withMotionMagicAcceleration(200).withMotionMagicCruiseVelocity(100);

    leader.getConfigurator().apply(config);
    follower.setControl(followReq);
    leader.setPosition(0);
    leader.getStatorCurrent().setUpdateFrequency(50);
    leader.getSupplyCurrent().setUpdateFrequency(50);
    leader.getVelocity().setUpdateFrequency(50);
    leader.getDutyCycle().setUpdateFrequency(100);
    leader.getMotorVoltage().setUpdateFrequency(50);
    leader.getSupplyVoltage().setUpdateFrequency(50);
    leader.optimizeBusUtilization();

    var encoderSettings =
        new CanandmagSettings()
            .setPositionFramePeriod(0.01)
            .setStatusFramePeriod(1)
            .setVelocityFramePeriod(0);
    encoder.setSettings(encoderSettings);
    initializePosition();
    setDefaultCommand(currentHome());
  }

  private void initializePosition() {
    if (useExternalPosition) {

      double initTime = Timer.getFPGATimestamp();
      // Make sure we've received an encoder update.
      while (encoder.getAbsPositionFrame().getTimestamp() < initTime) {
        if ((Timer.getFPGATimestamp() - initTime) > 0.1) {
          DriverStation.reportWarning("Elevator: encoder initialization failed.", false);
          return;
        }
        Timer.delay(0.02);
      }
      leader.setPosition(getExternalPosition());
    } else {
      leader.setPosition(0);
    }
  }

  public void setGoal(double position) {
    leader.setControl(posRequest.withPosition(position));
  }

  public Command setZero() {
    return runOnce(() -> leader.setPosition(0));
  }

  public Command positionCommand(double position) {
    return runOnce(() -> setGoal(position)).andThen(Commands.waitUntil((this::atGoal)));
  }

  public Command stowCommand() {
    return positionCommand(stowSetpoint);
  }

  public Command l2Command() {
    return positionCommand(l2Setpoint);
  }

  public Command l3Command() {
    return positionCommand(l3Setpoint);
  }

  public Command l4Command() {
    return positionCommand(l4Setpoint);
  }

  public Command relativePositionCommand(double relativePos) {
    return runOnce(() -> setGoal(posRequest.Position + relativePos));
  }

  public Command currentHome() {
    VoltageOut homeOutput = new VoltageOut(0);
    Debouncer currentDebounce = new Debouncer(0.125, DebounceType.kRising);
    Debouncer velocityDebounce = new Debouncer(0.125, DebounceType.kRising);

    return run(() -> leader.setControl(homeOutput.withOutput(-0.25).withIgnoreHardwareLimits(true)))
        .until(
            () ->
                currentDebounce.calculate(leader.getStatorCurrent().getValueAsDouble() > 5)
                    && velocityDebounce.calculate(Math.abs(getVelocity()) < 0.5))
        .finallyDo(
            (interrupted) -> {
              leader.setControl(homeOutput.withOutput(0));
              leader.setPosition(0);
              if (!interrupted) {
                removeDefaultCommand();
                leader.setControl(posRequest.withPosition(stowSetpoint));
              }
            });
  }

  public boolean isStowed() {
    return getPosition() < 3;
  }

  @Log
  public boolean atGoal() {
    return Math.abs(getPosition() - posRequest.Position) < 1 && Math.abs(getVelocity()) < 1;
  }

  @Log(key = "Velocity")
  public double getVelocity() {
    return leader.getVelocity().getValueAsDouble();
  }

  @Log(key = "Position")
  public double getPosition() {
    return leader.getPosition().getValueAsDouble();
  }

  @Log(key = "external encoder position")
  public double getExternalPosition() {
    return MathUtil.inputModulus(encoder.getAbsPosition(), -0.15, 0.85) * gearRatio;
  }

  @Override
  public void periodic() {

    log("Goal", posRequest.Position);
    log("StatorCurrent", leader.getStatorCurrent().getValueAsDouble());
    log("SupplyCurrent", leader.getSupplyCurrent().getValueAsDouble());
    log("OutputVoltage", leader.getMotorVoltage().getValueAsDouble());
    log("SupplyVoltage", leader.getSupplyVoltage().getValueAsDouble());
  }
}

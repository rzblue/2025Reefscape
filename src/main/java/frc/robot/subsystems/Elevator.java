package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

public class Elevator extends SubsystemBase implements Logged {
  private final TalonFX leader = new TalonFX(41);
  private final TalonFX follower = new TalonFX(42);

  private final Follower followReq = new Follower(leader.getDeviceID(), false);
  private final MotionMagicVoltage posRequest = new MotionMagicVoltage(0);

  private final Canandmag encoder = new Canandmag(1);

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
        .withReverseSoftLimitThreshold(0)
        .withReverseSoftLimitEnable(true);

    config.Slot0.withKP(3);
    config.MotionMagic.withMotionMagicAcceleration(100)
        .withMotionMagicCruiseVelocity(60);

    leader.getConfigurator().apply(config);
    follower.setControl(followReq);
    leader.setPosition(0);
    leader.getStatorCurrent().setUpdateFrequency(50);
    leader.getSupplyCurrent().setUpdateFrequency(50);
    leader.getVelocity().setUpdateFrequency(50);
    leader.getDutyCycle().setUpdateFrequency(100);
    leader.optimizeBusUtilization();

    var encoderSettings =
        new CanandmagSettings()
            .setPositionFramePeriod(0.01)
            .setStatusFramePeriod(1)
            .setVelocityFramePeriod(0);
    encoder.setSettings(encoderSettings);
    initializePosition();
  }

  private void initializePosition() {
    double initTime = Timer.getFPGATimestamp();
    // Make sure we've received an encoder update.
    /* 
    while (encoder.getPositionFrame().getTimestamp() < initTime) {
      if ((Timer.getFPGATimestamp() - initTime) > 0.1) {
        DriverStation.reportWarning("Elevator: encoder initialization failed.", false);
        return;
      }
      Timer.delay(0.02);
    }
    leader.setPosition(MathUtil.inputModulus(getExternalPosition(), -0.25, 0.75));
    */
    leader.setPosition(0);

  }

  public void setGoal(double position) {
    leader.setControl(posRequest.withPosition(position));
  }

  public Command setZero() {
    return runOnce(()-> leader.setPosition(0));
  }

  public Command positionCommand(double position) {
    return runOnce(() -> setGoal(position));
  }

  public Command relativePositionCommand(double relativePos) {
    return runOnce(() -> setGoal(posRequest.Position + relativePos));
  }

  public boolean isStowed() {
    return getPosition() < 3;
  }

  @Log(key = "Position")
  public double getPosition() {
    return leader.getPosition().getValueAsDouble();
  }

  @Log(key = "external encoder position")
  public double getExternalPosition() {
    return encoder.getAbsPosition() - 0.38836669921875;
  }

  @Override
  public void periodic() {
    log("Goal", posRequest.Position);
    log("Velocity", leader.getVelocity().getValueAsDouble());
    log("StatorCurrent", leader.getStatorCurrent().getValueAsDouble());
    log("SupplyCurrent", leader.getSupplyCurrent().getValueAsDouble());
  }
}

package frc.robot.subsystems;

import static frc.robot.Constants.AlgaeKickerConstants.*;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;

public class AlgaeKicker extends SubsystemBase implements Logged {
  private final SparkFlex motor = new SparkFlex(1, MotorType.kBrushless);

  private final REVLibError configureStatus;

  public AlgaeKicker() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kCoast);
    config.inverted(true);
    // Status 0: applied output, bus voltage, current, temperature
    config.signals.appliedOutputPeriodMs(20);
    // Status 1: faults + warnings
    config.signals.faultsAlwaysOn(true);
    config.signals.faultsPeriodMs(500);
    // Status 2: prim. encoder velocity
    config.signals.primaryEncoderVelocityAlwaysOn(true);
    config.signals.primaryEncoderVelocityPeriodMs(100);

    configureStatus =
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Log.Once(key = "init/ok")
  private boolean initializationOk() {
    log("init/config result", configureStatus.name());
    return configureStatus == REVLibError.kOk;
  }

  public void setOutput(double value) {
    motor.set(value);
  }

  public void stop() {
    setOutput(0);
  }

  public Command outputCommand(DoubleSupplier output) {
    return run(() -> setOutput(output.getAsDouble())).finallyDo(this::stop);
  }

  public Command outputCommand(double output) {
    return outputCommand(() -> output);
  }

  public Command kickAlgae() {
    return outputCommand(kickSpeed);
  }

  public Command extend() {
    return outputCommand(-1).withTimeout(0.5);
  }

  @Override
  public void periodic() {
    log("current", motor.getOutputCurrent());
    final double appliedOutput = motor.getAppliedOutput();
    final double busVoltage = motor.getBusVoltage();
    log("output voltage", appliedOutput * busVoltage);
    log("bus voltage", busVoltage);
    log("temperature", motor.getMotorTemperature());
    log("faults", motor.getFaults().rawBits);
    log("warnings", motor.getWarnings().rawBits);
  }
}

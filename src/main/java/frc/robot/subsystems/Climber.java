package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

public class Climber extends SubsystemBase implements Logged {
  private Servo servo = new Servo(servoPort);

  private final double latchedPosition = 0;
  private final double unlatchedPosition = 1;

  public Climber() {
    servo.setBoundsMicroseconds(1800, 0, 0, 0, 1450);
    // Preload PWM with reset value
    reset();
  }

  private void setPosition(double position) {
    servo.set(position);
  }

  public void reset() {
    setPosition(latchedPosition);
  }

  public void deploy() {
    setPosition(unlatchedPosition);
  }

  @Log(key = "latched")
  public boolean isLatched() {
    return servo.get() == latchedPosition;
  }
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private Servo servo = new Servo(0);

    public Climber() {
        servo.setBoundsMicroseconds(1800, 0, 0, 0, 1450);

    }
    public void latch() {
        servo.set(0);
        
    }
    public void unlatch() {
        servo.set(1);
    }
    
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SmartLogger;
import java.util.Random;

// Blinkin PWM LED controller. Note: PWM actuators are disabled by RoboRIO safety during robot-disabled
// mode, so LED patterns only take effect when the robot is enabled (teleop/auto).
public class LEDSubsystem extends SubsystemBase {
  private final Spark blinkin;
  private final Random random = new Random();
  
  public static class Pattern {
    public static final double RED = 0.61;
    public static final double ORANGE = 0.65;
    public static final double GREEN = 0.77;
    public static final double WHITE = 0.93;
    public static final double LAVA = -0.39;
    public static final double OFF = 0.99;
    public static final double MODE_12V = 1.09;
    
    private static final double[] ALL_COLORS = {RED, ORANGE, GREEN, WHITE};
    public static double getRandomColor(Random rng) {
      return ALL_COLORS[rng.nextInt(ALL_COLORS.length)];
    }
  }

  public LEDSubsystem() {
    this.blinkin = new Spark(Constants.BLINKIN_PWM_PORT);
    
    // Configure for 12V strip mode
    blinkin.set(Pattern.MODE_12V);
    
    // Set default pattern (will only work when enabled)
    blinkin.set(Pattern.LAVA);
    
    SmartLogger.logConsole("Blinkin LED initialized on PWM port " + Constants.BLINKIN_PWM_PORT, "LED");
    SmartLogger.logConsole("NOTE: PWM actuators disabled during disabled mode (RoboRIO safety)", "LED");
  }
  
  public void setPattern(double sparkValue) {
    blinkin.set(sparkValue);
  }
  
  public void setRandomColor() {
    double color = Pattern.getRandomColor(random);
    blinkin.set(color);
    SmartLogger.logConsole("LED color changed (random): " + color, "LED");
  }
  
  public void setOff() { blinkin.set(Pattern.OFF); }
  public void setRed() { blinkin.set(Pattern.RED); }
  public void setGreen() { blinkin.set(Pattern.GREEN); }
  public void setLava() { blinkin.set(Pattern.LAVA); }
  public void setFire() { blinkin.set(-0.49); }
  public void setRainbow() { blinkin.set(-0.97); }
  
  @Override
  public void periodic() {
    // Periodic updates can go here if needed
  }
}
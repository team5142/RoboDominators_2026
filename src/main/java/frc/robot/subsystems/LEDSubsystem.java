package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.SmartLogger;
import java.util.Random;

/**
 * LED subsystem for Blinkin controller
 * 
 * IMPORTANT LIMITATION:
 * - Blinkin is a PWM actuator
 * - PWM actuators are DISABLED by RoboRIO safety during disabled mode
 * - LED commands will ONLY work when robot is ENABLED (teleop/auto)
 * - Pre-match calibration LEDs are NOT POSSIBLE with this hardware
 * 
 * Future work:
 * - Consider CAN-based LED controller if disabled-mode operation is needed
 * - Or use indicator lights that aren't controlled by Blinkin
 */
public class LEDSubsystem extends SubsystemBase {
  private final RobotState robotState;
  private final TagVisionSubsystem tagVisionSubsystem;
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

  public LEDSubsystem(RobotState robotState, TagVisionSubsystem tagVisionSubsystem) {
    this.robotState = robotState;
    this.tagVisionSubsystem = tagVisionSubsystem;
    this.blinkin = new Spark(Constants.BLINKIN_PWM_PORT);
    
    // Configure for 12V strip mode
    blinkin.set(Pattern.MODE_12V);
    try {
      Thread.sleep(100);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
    
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
package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Tunable PathPlanner PID gains using AdvantageKit's LoggedNetworkNumber
 * 
 * These can be tuned live in AdvantageScope's Tuning Mode without rebuilding code.
 * Values persist across reboots.
 * 
 * See: https://docs.advantagescope.org/overview/live-sources/tuning-mode/
 */
public class TunablePathPlannerPID {
  
  // Translation PID
  public static final LoggedNetworkNumber TRANSLATION_KP = 
      new LoggedNetworkNumber("PathPlanner/Translation/kP", Constants.Auto.TRANSLATION_KP);
  public static final LoggedNetworkNumber TRANSLATION_KI = 
      new LoggedNetworkNumber("PathPlanner/Translation/kI", Constants.Auto.TRANSLATION_KI);
  public static final LoggedNetworkNumber TRANSLATION_KD = 
      new LoggedNetworkNumber("PathPlanner/Translation/kD", Constants.Auto.TRANSLATION_KD);
  
  // Rotation PID
  public static final LoggedNetworkNumber ROTATION_KP = 
      new LoggedNetworkNumber("PathPlanner/Rotation/kP", Constants.Auto.ROTATION_KP);
  public static final LoggedNetworkNumber ROTATION_KI = 
      new LoggedNetworkNumber("PathPlanner/Rotation/kI", Constants.Auto.ROTATION_KI);
  public static final LoggedNetworkNumber ROTATION_KD = 
      new LoggedNetworkNumber("PathPlanner/Rotation/kD", Constants.Auto.ROTATION_KD);
  
  // Track last values to detect changes
  private static double lastTranslationKP = Constants.Auto.TRANSLATION_KP;
  private static double lastTranslationKI = Constants.Auto.TRANSLATION_KI;
  private static double lastTranslationKD = Constants.Auto.TRANSLATION_KD;
  private static double lastRotationKP = Constants.Auto.ROTATION_KP;
  private static double lastRotationKI = Constants.Auto.ROTATION_KI;
  private static double lastRotationKD = Constants.Auto.ROTATION_KD;
  
  /**
   * Check if any PID value changed (for re-configuration)
   */
  public static boolean hasChanged() {
    double currentTranslationKP = TRANSLATION_KP.get();
    double currentTranslationKI = TRANSLATION_KI.get();
    double currentTranslationKD = TRANSLATION_KD.get();
    double currentRotationKP = ROTATION_KP.get();
    double currentRotationKI = ROTATION_KI.get();
    double currentRotationKD = ROTATION_KD.get();
    
    boolean changed = 
        currentTranslationKP != lastTranslationKP ||
        currentTranslationKI != lastTranslationKI ||
        currentTranslationKD != lastTranslationKD ||
        currentRotationKP != lastRotationKP ||
        currentRotationKI != lastRotationKI ||
        currentRotationKD != lastRotationKD;
    
    if (changed) {
      lastTranslationKP = currentTranslationKP;
      lastTranslationKI = currentTranslationKI;
      lastTranslationKD = currentTranslationKD;
      lastRotationKP = currentRotationKP;
      lastRotationKI = currentRotationKI;
      lastRotationKD = currentRotationKD;
    }
    
    return changed;
  }
}

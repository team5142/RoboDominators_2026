package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Tunable CTRE motor gains using AdvantageKit's LoggedNetworkNumber
 * 
 * These can be tuned live in AdvantageScope's Tuning Mode without rebuilding code.
 * Values persist across reboots.
 * 
 * See: https://docs.advantagescope.org/overview/live-sources/tuning-mode/
 */
public class TunableCTREGains {
  
  // ===== STEER MOTOR GAINS (SysId-tuned) =====
  public static final LoggedNetworkNumber STEER_KP = 
      new LoggedNetworkNumber("CTRE/Steer/kP", Constants.Swerve.SteerGains.kP);
  public static final LoggedNetworkNumber STEER_KI = 
      new LoggedNetworkNumber("CTRE/Steer/kI", Constants.Swerve.SteerGains.kI);
  public static final LoggedNetworkNumber STEER_KD = 
      new LoggedNetworkNumber("CTRE/Steer/kD", Constants.Swerve.SteerGains.kD);
  public static final LoggedNetworkNumber STEER_KS = 
      new LoggedNetworkNumber("CTRE/Steer/kS", Constants.Swerve.SteerGains.kS);
  public static final LoggedNetworkNumber STEER_KV = 
      new LoggedNetworkNumber("CTRE/Steer/kV", Constants.Swerve.SteerGains.kV);
  public static final LoggedNetworkNumber STEER_KA = 
      new LoggedNetworkNumber("CTRE/Steer/kA", Constants.Swerve.SteerGains.kA);
  
  // ===== DRIVE MOTOR GAINS (SysId-tuned) =====
  public static final LoggedNetworkNumber DRIVE_KP = 
      new LoggedNetworkNumber("CTRE/Drive/kP", Constants.Swerve.DriveGains.kP);
  public static final LoggedNetworkNumber DRIVE_KI = 
      new LoggedNetworkNumber("CTRE/Drive/kI", Constants.Swerve.DriveGains.kI);
  public static final LoggedNetworkNumber DRIVE_KD = 
      new LoggedNetworkNumber("CTRE/Drive/kD", Constants.Swerve.DriveGains.kD);
  public static final LoggedNetworkNumber DRIVE_KS = 
      new LoggedNetworkNumber("CTRE/Drive/kS", Constants.Swerve.DriveGains.kS);
  public static final LoggedNetworkNumber DRIVE_KV = 
      new LoggedNetworkNumber("CTRE/Drive/kV", Constants.Swerve.DriveGains.kV);
  public static final LoggedNetworkNumber DRIVE_KA =  // NEW: Added Drive kA
      new LoggedNetworkNumber("CTRE/Drive/kA", Constants.Swerve.DriveGains.kA);
  
  // Track last values to detect changes
  private static double lastSteerKP = Constants.Swerve.SteerGains.kP;
  private static double lastSteerKD = Constants.Swerve.SteerGains.kD;
  private static double lastDriveKP = Constants.Swerve.DriveGains.kP;
  private static double lastDriveKV = Constants.Swerve.DriveGains.kV;
  
  /**
   * Check if any gain value changed (for re-configuration)
   */
  public static boolean hasChanged() {
    double currentSteerKP = STEER_KP.get();
    double currentSteerKD = STEER_KD.get();
    double currentDriveKP = DRIVE_KP.get();
    double currentDriveKV = DRIVE_KV.get();
    
    boolean changed = 
        currentSteerKP != lastSteerKP ||
        currentSteerKD != lastSteerKD ||
        currentDriveKP != lastDriveKP ||
        currentDriveKV != lastDriveKV;
    
    if (changed) {
      lastSteerKP = currentSteerKP;
      lastSteerKD = currentSteerKD;
      lastDriveKP = currentDriveKP;
      lastDriveKV = currentDriveKV;
    }
    
    return changed;
  }
}

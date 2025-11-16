package frc.robot;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;

public class RobotState {
  public enum Mode {
    DISABLED,
    TELEOP,
    AUTO,
    TEST
  }

  public enum AssistMode {
    NONE,
    ALIGN_TO_TARGET,
    AUTO_SCORE,
    FOLLOW_PREPLANNED_PATH
  }

  private Mode mode = Mode.DISABLED;
  private AssistMode assistMode = AssistMode.NONE;
  private boolean enabled = false;
  private Pose2d robotPose = new Pose2d();

  public void setRobotPose(Pose2d pose) {
    this.robotPose = pose;
  }

  public Pose2d getRobotPose() {
    return robotPose;
  }

  public void setMode(Mode mode) {
    this.mode = mode;
    log();
  }

  public Mode getMode() {
    return mode;
  }

  public void setAssistMode(AssistMode assistMode) {
    this.assistMode = assistMode;
    log();
  }

  public AssistMode getAssistMode() {
    return assistMode;
  }

  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
    log();
  }

  public boolean isEnabled() {
    return enabled;
  }

  private void log() {
    Logger.recordOutput("RobotState/Mode", mode.toString());
    Logger.recordOutput("RobotState/AssistMode", assistMode.toString());
    Logger.recordOutput("RobotState/Enabled", enabled);
  }
}

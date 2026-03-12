package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import frc.robot.RobotState.ShootingZone;

// Tracks the current Rebuilt game phase and whether our hub is active.
//
// Phase timing (DriverStation.getMatchTime() counts DOWN during teleop from ~140):
//   TRANSITION_SHIFT:  140 - 130s  (10s, both hubs active)
//   SHIFT_1:           130 - 105s  (25s)
//   SHIFT_2:           105 -  80s  (25s)
//   SHIFT_3:            80 -  55s  (25s)
//   SHIFT_4:            55 -  30s  (25s)
//   END_GAME:           30 -   0s  (30s, both hubs active)
//
// Game data: 'R' = Red hub inactive first (Red inactive in shifts 1+3, active in 2+4).
//            'B' = Blue hub inactive first (Blue inactive in shifts 1+3, active in 2+4).
// Data arrives ~3s into teleop (after auto fuel scoring is tallied).
// There is a ~1s gap between auto ending and teleop starting - getMatchTime() will
// briefly return a value >140 at teleop start; this is handled in computePhase().
//
// Call update() once per periodic loop (all modes).
//
// ============================================================
// AUTOFIRE REGULATION STRATEGY (to implement when turret/singulator are ready)
// ============================================================
//
// MECHANISM COUPLING RULES:
//   - Flywheels (2x Kraken x60): always running at minimum shooting speed.
//   - Singulator: the fire gate. Singulator ON = shooting. Singulator OFF = holding.
//   - Spindexer: always coupled with singulator (feeds it). Turn both on together.
//   - Intake: separate logic - do NOT couple intake to singulator state.
//
// FIRE DECISION TREE (evaluated in priority order, highest first):
//
//   Priority 1 - GLOBAL DISABLE FLAG
//     If a "mechanism broken" sticky flag is set, singulator stays OFF regardless.
//     This should be a persistent/sticky toggle (survives disable/enable cycles).
//     Operator sets via dashboard or button. Implement as a boolean in RobotState.
//     Example: robotState.setAutofireDisabled(true)
//
//   Priority 2 - ZONE
//     Opponent zone:  NEVER fire. No hub target exists on that side.
//     Neutral zone:   Fire only to PASS back to alliance zone (pass targets = mirrored
//                     versions of hub scoring targets on the alliance side).
//     Alliance zone:  Fire to SCORE into hub (normal shooting mode).
//     Zone is determined from PoseEstimatorSubsystem.getEstimatedPose() vs field geometry.
//
//   Priority 3 - HUB ACTIVE
//     In alliance zone:  only fire when isHubActive() == true.
//     In neutral zone:   only pass when the NEXT active shift is coming up soon (see P4).
//     In END_GAME both hubs are always active - always allow firing.
//
//   Priority 4 - STOP PASSING (neutral zone fill-up logic)
//     If we are in the opponent's scoring period (hub inactive for us) AND we are in
//     the neutral zone: STOP passing when the current inactive shift is ending soon.
//     Goal: arrive at the alliance zone with a full hopper when our hub activates.
//
//     Stop-pass threshold: getSecondsUntilPhaseEnd() < TRANSIT_TO_ALLIANCE_SEC + PASS_STOP_BUFFER_SEC
//       TRANSIT_TO_ALLIANCE_SEC: tunable - time to drive from neutral zone to alliance hub (est. 4-5s)
//       PASS_STOP_BUFFER_SEC:    tunable - extra buffer to fill hopper before transit (est. 1-2s)
//       Combined default estimate: ~6s before shift end, stop passing in neutral zone.
//
//     Example: shift ends at 105s, TRANSIT=4s, BUFFER=2s -> stop passing when matchTime < 111s.
//     Use: !isHubActive() && inNeutralZone && getSecondsUntilPhaseEnd() < (TRANSIT + BUFFER)
//
//   Priority 5 - OPERATOR HOLD OVERRIDE
//     Operator can suppress singulator at any time (e.g., jammed ball, physical issue).
//     Decide with drive team: latching toggle (one press on, one press off) vs momentary hold.
//     Recommend toggle for match use. Implement as a sticky boolean in RobotState.
//
// SUMMARY - pseudocode for AutofireController.shouldFire():
//   if (robotState.isAutofireDisabled())       return false;  // P1: broken mechanism
//   if (inOpponentZone())                      return false;  // P2: wrong zone
//   if (robotState.isOperatorHoldActive())     return false;  // P5: driver override
//   if (!isHubActive() && !inNeutralZone())    return false;  // P3: hub not active
//   if (inNeutralZone() && stopPassingNow())   return false;  // P4: fill up before transit
//   if (!isHubActive() && !isHubActiveIn(SPINUP_LEAD_SEC)) return false; // P3: not soon enough
//   return true;
//
// PASSING TARGETS:
//   Neutral zone pass targets are predefined field poses, mirrored by alliance (same
//   infrastructure as hub scoring targets). The turret aims at these during neutral pass mode.
// ============================================================
public class MatchPhaseTracker {

  private static final double SPINUP_LEAD_SEC   = 2.0;
  private static final double STOP_FEED_SEC     = 0.5;
  // How long into each inactive period we allow neutral-zone passing.
  private static final double NEUTRAL_PASS_WINDOW_SEC = 15.0;

  public enum GamePhase {
    AUTO,
    TRANSITION_SHIFT,
    SHIFT_1,
    SHIFT_2,
    SHIFT_3,
    SHIFT_4,
    END_GAME,
    DISABLED
  }

  private static final double SHIFT_1_START       = 130.0;
  private static final double SHIFT_2_START       = 105.0;
  private static final double SHIFT_3_START       =  80.0;
  private static final double SHIFT_4_START       =  55.0;
  private static final double END_GAME_START      =  30.0;
  private static final double END_GAME_15_WARNING =  15.0;

  private GamePhase currentPhase = GamePhase.DISABLED;
  private boolean hubActive = true;
  private boolean redInactiveFirst = false;
  private boolean gameDataReceived = false;
  private double lastLoggedTime = -1.0;

  // Match time recorded when our active period ended — used to open neutral pass window each inactive period.
  // Reset to -1 at start of each of our active periods so the window is fresh when the period ends.
  private double ourPeriodEndMatchTime = -1.0;
  private boolean neutralPassWindowOpen = false;

  private int lastAnnouncedSecond = -1;
  private boolean announcedEndGame15 = false;
  private boolean announcedEndGameEnd = false;
  private boolean announcedSpinupOpen = false;
  private boolean announcedShootOpen = false;

  // Call every loop in all modes.
  public void update() {
    if (DriverStation.isAutonomousEnabled()) {
      currentPhase = GamePhase.AUTO;
      hubActive = true;
      return;
    }

    if (!DriverStation.isTeleopEnabled()) {
      if (currentPhase != GamePhase.DISABLED) {
        currentPhase = GamePhase.DISABLED;
        resetSimCommentaryState();
      }
      hubActive = true;
      return;
    }

    refreshGameData();
    double matchTime = DriverStation.getMatchTime();
    GamePhase newPhase = computePhase(matchTime);
    boolean newHubActive = computeHubActive(newPhase);

    if (newPhase != currentPhase) {
      onPhaseEnter(newPhase, matchTime, newHubActive);
      currentPhase = newPhase;
    }

    hubActive = newHubActive;
    updateNeutralPassWindow(newHubActive, matchTime);
    runSimCommentary(matchTime, newHubActive);
    logToAdvantageKit(matchTime);
  }

  public boolean isHubActive() { return hubActive; }

  public boolean isHubActiveIn(double leadSeconds) {
    if (hubActive) return true;
    double futureTime = DriverStation.getMatchTime() - leadSeconds;
    return computeHubActive(computePhase(futureTime));
  }

  public boolean shouldShoot(double leadSeconds, double stopEarlySeconds) {
    if (!DriverStation.isTeleopEnabled()) return false;
    if (!isHubActiveIn(leadSeconds)) return false;
    if (hubActive && stopEarlySeconds > 0.0) {
      double futureTime = DriverStation.getMatchTime() - stopEarlySeconds;
      if (!computeHubActive(computePhase(futureTime))) return false;
    }
    return true;
  }

  // Zone-aware shoot gate — only active when match time is valid (FMS or Practice mode).
  // Plain teleop (getMatchTime() == -1) always returns true (no suppression).
  public boolean shouldShootInZone(ShootingZone zone) {
    double matchTime = DriverStation.getMatchTime();
    if (matchTime < 0) return true; // plain teleop — no suppression

    switch (zone) {
      case OPPONENT:
        return false;
      case ALLIANCE:
        // Shoot freely when hub is active, TRANSITION, or END_GAME. Suppressed during opponent periods.
        return hubActive;
      case NEUTRAL:
        // Pass back only within the first 15s of each inactive period.
        // During TRANSITION or END_GAME neutral passing is button-gated (not handled here — always false).
        if (currentPhase == GamePhase.TRANSITION_SHIFT || currentPhase == GamePhase.END_GAME) return false;
        return !hubActive && neutralPassWindowOpen;
      default:
        return false;
    }
  }

  public GamePhase getPhase()    { return currentPhase; }
  public double getMatchTime()   { return DriverStation.getMatchTime(); }
  public boolean hasGameData()   { return gameDataReceived; }

  // Seconds until current phase ends (0 if not in a timed phase).
  public double getSecondsUntilPhaseEnd() {
    double remaining = DriverStation.getMatchTime() - phaseEndTime(currentPhase);
    return remaining > 0 ? remaining : 0.0;
  }

  // 1-4 during SHIFT_1-4, 0 otherwise.
  public int getShiftNumber() {
    return switch (currentPhase) {
      case SHIFT_1 -> 1;
      case SHIFT_2 -> 2;
      case SHIFT_3 -> 3;
      case SHIFT_4 -> 4;
      default -> 0;
    };
  }

  // Human-readable phase name for dashboard display.
  public String getPhaseName() {
    return switch (currentPhase) {
      case AUTO             -> "AUTO";
      case TRANSITION_SHIFT -> "TRANSITION";
      case SHIFT_1          -> "SHIFT 1";
      case SHIFT_2          -> "SHIFT 2";
      case SHIFT_3          -> "SHIFT 3";
      case SHIFT_4          -> "SHIFT 4";
      case END_GAME         -> "END GAME";
      case DISABLED         -> "DISABLED";
    };
  }

  // ---- Private helpers ----

  // Tracks when our hub transitions active→inactive so we can open a 15s neutral pass window.
  // Resets the window when we enter an active period (so each inactive period gets a fresh 15s).
  private boolean wasHubActive = true;
  private void updateNeutralPassWindow(boolean hubActiveNow, double matchTime) {
    if (hubActiveNow && !wasHubActive) {
      // Our period just ended — reset so next inactive period gets a fresh window
      neutralPassWindowOpen = false;
      ourPeriodEndMatchTime = -1.0;
    } else if (!hubActiveNow && wasHubActive) {
      // Opponent period just started — open the pass window and record the start time
      ourPeriodEndMatchTime = matchTime;
      neutralPassWindowOpen = true;
    } else if (!hubActiveNow && neutralPassWindowOpen && ourPeriodEndMatchTime > 0) {
      // Check if 15s have elapsed since the inactive period began
      double elapsed = ourPeriodEndMatchTime - matchTime; // matchTime counts DOWN
      if (elapsed >= NEUTRAL_PASS_WINDOW_SEC) {
        neutralPassWindowOpen = false;
        SmartLogger.logConsole("GAMEPHASES: Neutral pass window closed — collect mode", "GAMEPHASES");
      }
    }
    wasHubActive = hubActiveNow;
    Logger.recordOutput("MatchPhase/NeutralPassWindowOpen", neutralPassWindowOpen);
  }

  private void onPhaseEnter(GamePhase phase, double matchTime, boolean hubActiveNow) {
    String t = String.format("%.1f", matchTime);
    switch (phase) {
      case TRANSITION_SHIFT -> {
        SmartLogger.logConsole(
            "GAMEPHASES: === TELEOP START - Transition Shift === t=" + t
            + "s | Both hubs ACTIVE | Waiting for game data...", "GAMEPHASES");
        resetSimCommentaryState();
      }
      case SHIFT_1 -> {
        String owner = hubActiveNow ? "WE ARE ACTIVE - SHOOT" : "WE ARE INACTIVE - hold fire";
        SmartLogger.logConsole(
            "GAMEPHASES: === SHIFT 1 START === t=" + t + "s | " + owner + " | 25s window",
            "GAMEPHASES");
        resetShiftCommentaryState();
      }
      case SHIFT_2 -> {
        String owner = hubActiveNow ? "WE ARE ACTIVE - SHOOT" : "WE ARE INACTIVE - hold fire";
        SmartLogger.logConsole(
            "GAMEPHASES: === SHIFT 2 START === t=" + t + "s | " + owner + " | 25s window",
            "GAMEPHASES");
        resetShiftCommentaryState();
      }
      case SHIFT_3 -> {
        String owner = hubActiveNow ? "WE ARE ACTIVE - SHOOT" : "WE ARE INACTIVE - hold fire";
        SmartLogger.logConsole(
            "GAMEPHASES: === SHIFT 3 START === t=" + t + "s | " + owner + " | 25s window",
            "GAMEPHASES");
        resetShiftCommentaryState();
      }
      case SHIFT_4 -> {
        String owner = hubActiveNow ? "WE ARE ACTIVE - SHOOT" : "WE ARE INACTIVE - hold fire";
        SmartLogger.logConsole(
            "GAMEPHASES: === SHIFT 4 START === t=" + t + "s | " + owner + " | 25s window",
            "GAMEPHASES");
        resetShiftCommentaryState();
      }
      case END_GAME -> {
        SmartLogger.logConsole(
            "GAMEPHASES: === END GAME START === t=" + t + "s | Both hubs ACTIVE | 30s remaining",
            "GAMEPHASES");
        announcedEndGame15 = false;
        announcedEndGameEnd = false;
      }
      default -> { }
    }
  }

  private void runSimCommentary(double matchTime, boolean hubActiveNow) {
    int secondsRemaining = (int) matchTime;
    if (secondsRemaining == lastAnnouncedSecond) return;
    lastAnnouncedSecond = secondsRemaining;

    switch (currentPhase) {
      case TRANSITION_SHIFT -> announceTransitionShift(matchTime);
      case SHIFT_1, SHIFT_2, SHIFT_3, SHIFT_4 -> announceShift(matchTime, hubActiveNow);
      case END_GAME -> announceEndGame(matchTime);
      default -> { }
    }
  }

  private void announceTransitionShift(double matchTime) {
    double untilShift1 = matchTime - SHIFT_1_START;
    int sec = (int) matchTime;
    if (!gameDataReceived && sec == 138) {
      SmartLogger.logConsole(
          "GAMEPHASES: Transition | Waiting for game data... "
          + String.format("%.0f", untilShift1) + "s until Shift 1", "GAMEPHASES");
    }
    if (sec == 133 || sec == 132 || sec == 131) {
      SmartLogger.logConsole(
          "GAMEPHASES: Shift 1 in " + String.format("%.0f", untilShift1) + "s", "GAMEPHASES");
    }
  }

  private void announceShift(double matchTime, boolean hubActiveNow) {
    double phaseEnd = phaseEndTime(currentPhase);
    double timeLeft = matchTime - phaseEnd;

    if (!hubActiveNow && !announcedSpinupOpen) {
      GamePhase next = nextPhase(currentPhase);
      if (computeHubActive(next) && timeLeft <= SPINUP_LEAD_SEC + 0.5) {
        SmartLogger.logConsole(
            "GAMEPHASES: SPINUP NOW - hub active in " + String.format("%.1f", timeLeft) + "s",
            "GAMEPHASES");
        announcedSpinupOpen = true;
      }
    }

    if (hubActiveNow && !announcedShootOpen) {
      SmartLogger.logConsole(
          "GAMEPHASES: SHOOT WINDOW OPEN | " + String.format("%.0f", timeLeft) + "s remaining",
          "GAMEPHASES");
      announcedShootOpen = true;
    }

    if (hubActiveNow && timeLeft <= STOP_FEED_SEC + 0.5 && timeLeft > 0) {
      SmartLogger.logConsole(
          "GAMEPHASES: STOP FEEDING - window closes in " + String.format("%.1f", timeLeft) + "s",
          "GAMEPHASES");
    }

    if (timeLeft <= 3.5 && timeLeft > 0) {
      SmartLogger.logConsole(
          "GAMEPHASES: Phase ends in " + (int) Math.ceil(timeLeft) + "...", "GAMEPHASES");
    }

    if (!hubActiveNow && (int) matchTime % 5 == 0) {
      GamePhase nextActive = nextActivePhase();
      if (nextActive != GamePhase.DISABLED) {
        double untilActive = matchTime - phaseEndTime(prevPhaseOf(nextActive));
        SmartLogger.logConsole(
            "GAMEPHASES: Next active window (" + nextActive + ") in "
            + String.format("%.0f", untilActive) + "s", "GAMEPHASES");
      }
    }
  }

  private void announceEndGame(double matchTime) {
    if (!announcedEndGame15 && matchTime <= END_GAME_15_WARNING) {
      SmartLogger.logConsole(
          "GAMEPHASES: === END GAME 15s WARNING === " + (int) matchTime + "s left", "GAMEPHASES");
      announcedEndGame15 = true;
    }
    if (!announcedEndGameEnd && matchTime <= 3.5 && matchTime > 0) {
      SmartLogger.logConsole(
          "GAMEPHASES: Match ends in " + (int) Math.ceil(matchTime) + "...", "GAMEPHASES");
      if ((int) Math.ceil(matchTime) == 1) announcedEndGameEnd = true;
    }
  }

  private void refreshGameData() {
    if (gameDataReceived) return;
    String data = DriverStation.getGameSpecificMessage();
    if (data == null || data.isEmpty()) return;
    boolean isRed = DriverStation.getAlliance().map(a -> a == Alliance.Red).orElse(false);
    switch (data.charAt(0)) {
      case 'R' -> {
        redInactiveFirst = true;
        gameDataReceived = true;
        String weWon  = isRed ? "WE WON AUTO" : "OPPONENT WON AUTO";
        String shifts = isRed ? "2+4" : "1+3";
        SmartLogger.logConsole(
            "GAMEPHASES: Game data - RED hub inactive first | " + weWon
            + " | We score in shifts " + shifts, "GAMEPHASES");
      }
      case 'B' -> {
        redInactiveFirst = false;
        gameDataReceived = true;
        String weWon  = isRed ? "OPPONENT WON AUTO" : "WE WON AUTO";
        String shifts = isRed ? "1+3" : "2+4";
        SmartLogger.logConsole(
            "GAMEPHASES: Game data - BLUE hub inactive first | " + weWon
            + " | We score in shifts " + shifts, "GAMEPHASES");
      }
      default -> SmartLogger.logConsole(
          "GAMEPHASES: Game data corrupt: '" + data.charAt(0) + "'", "GAMEPHASES");
    }
    Logger.recordOutput("MatchPhase/GameData", data);
    Logger.recordOutput("MatchPhase/RedInactiveFirst", redInactiveFirst);
  }

  private GamePhase computePhase(double matchTime) {
    // matchTime is -1 when DS is in teleop without FMS - treat as TRANSITION_SHIFT
    if (matchTime < 0)              return GamePhase.TRANSITION_SHIFT;
    if (matchTime > SHIFT_1_START)  return GamePhase.TRANSITION_SHIFT;
    if (matchTime > SHIFT_2_START)  return GamePhase.SHIFT_1;
    if (matchTime > SHIFT_3_START)  return GamePhase.SHIFT_2;
    if (matchTime > SHIFT_4_START)  return GamePhase.SHIFT_3;
    if (matchTime > END_GAME_START) return GamePhase.SHIFT_4;
    return GamePhase.END_GAME;
  }

  private boolean computeHubActive(GamePhase phase) {
    if (phase == GamePhase.AUTO || phase == GamePhase.TRANSITION_SHIFT
        || phase == GamePhase.END_GAME || phase == GamePhase.DISABLED) {
      return true;
    }
    if (!gameDataReceived) return true;
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();
    if (allianceOpt.isEmpty()) return true;
    boolean isRed = allianceOpt.get() == Alliance.Red;
    boolean shift1Active = isRed ? !redInactiveFirst : redInactiveFirst;
    return switch (phase) {
      case SHIFT_1 -> shift1Active;
      case SHIFT_2 -> !shift1Active;
      case SHIFT_3 -> shift1Active;
      case SHIFT_4 -> !shift1Active;
      default -> true;
    };
  }

  private static double phaseEndTime(GamePhase phase) {
    return switch (phase) {
      case TRANSITION_SHIFT -> SHIFT_1_START;
      case SHIFT_1          -> SHIFT_2_START;
      case SHIFT_2          -> SHIFT_3_START;
      case SHIFT_3          -> SHIFT_4_START;
      case SHIFT_4          -> END_GAME_START;
      case END_GAME         -> 0.0;
      default               -> 0.0;
    };
  }

  private static GamePhase nextPhase(GamePhase phase) {
    return switch (phase) {
      case TRANSITION_SHIFT -> GamePhase.SHIFT_1;
      case SHIFT_1          -> GamePhase.SHIFT_2;
      case SHIFT_2          -> GamePhase.SHIFT_3;
      case SHIFT_3          -> GamePhase.SHIFT_4;
      case SHIFT_4          -> GamePhase.END_GAME;
      default               -> GamePhase.DISABLED;
    };
  }

  private static GamePhase prevPhaseOf(GamePhase phase) {
    return switch (phase) {
      case SHIFT_1  -> GamePhase.TRANSITION_SHIFT;
      case SHIFT_2  -> GamePhase.SHIFT_1;
      case SHIFT_3  -> GamePhase.SHIFT_2;
      case SHIFT_4  -> GamePhase.SHIFT_3;
      case END_GAME -> GamePhase.SHIFT_4;
      default       -> GamePhase.DISABLED;
    };
  }

  private GamePhase nextActivePhase() {
    GamePhase check = nextPhase(currentPhase);
    for (int i = 0; i < 5; i++) {
      if (check == GamePhase.DISABLED) break;
      if (computeHubActive(check)) return check;
      check = nextPhase(check);
    }
    return GamePhase.DISABLED;
  }

  private void resetSimCommentaryState() {
    lastAnnouncedSecond = -1;
    announcedEndGame15 = false;
    announcedEndGameEnd = false;
    resetShiftCommentaryState();
  }

  private void resetShiftCommentaryState() {
    announcedSpinupOpen = false;
    announcedShootOpen = false;
  }

  private void logToAdvantageKit(double matchTime) {
    if (Math.abs(matchTime - lastLoggedTime) < 1.0) return;
    lastLoggedTime = matchTime;
    Logger.recordOutput("MatchPhase/Phase", currentPhase.toString());
    Logger.recordOutput("MatchPhase/HubActive", hubActive);
    Logger.recordOutput("MatchPhase/MatchTime", matchTime);
    Logger.recordOutput("MatchPhase/GameDataReceived", gameDataReceived);
    Logger.recordOutput("MatchPhase/RedInactiveFirst", redInactiveFirst);
  }
}

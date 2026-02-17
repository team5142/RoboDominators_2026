package frc.robot.util;

import static frc.robot.Constants.StartingPositions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.commands.drive.SmartDriveToPosition;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.InetSocketAddress;
import com.sun.net.httpserver.HttpServer;
import com.sun.net.httpserver.HttpExchange;
import java.util.concurrent.Executors;

// Touchscreen operator interface - subscribes to NetworkTables commands from HTML dashboard
public class TouchscreenInterface {

  private final RobotState robotState;
  private final DriveSubsystem driveSubsystem;
  private final PoseEstimatorSubsystem poseEstimator;
  private final QuestNavSubsystem questNav;

  private final NetworkTableInstance ntInst = NetworkTableInstance.getDefault();

  private Command activeOperatorDrive = null;

  private static class SmartDriveTarget {
    public final Pose2d staging;
    public final Pose2d precise;

    public SmartDriveTarget(Pose2d staging, Pose2d precise) {
      this.staging = staging;
      this.precise = precise;
    }
  }

  private final Map<String, SmartDriveTarget> targets = new HashMap<>();
  private final Map<String, Boolean> lastValueByKey = new HashMap<>();
  private final Map<String, BooleanPublisher> publishers = new HashMap<>();
  private String lastCommandPosition = "";

  private HttpServer httpServer;

  public TouchscreenInterface(
      RobotState robotState,
      DriveSubsystem driveSubsystem,
      PoseEstimatorSubsystem poseEstimator,
      GyroSubsystem gyro,
      QuestNavSubsystem questNav) {

    this.robotState = robotState;
    this.driveSubsystem = driveSubsystem;
    this.poseEstimator = poseEstimator;
    this.questNav = questNav;
  }

  public void configure() {


    // Keep existing boolean listeners for AdvantageScope/Shuffleboard compatibility
    NetworkTable opTable = ntInst.getTable("OperatorInterface");
    NetworkTable driveTable = opTable.getSubTable("DriveToPosition");

    targets.forEach((key, target) -> {
      BooleanTopic topic = driveTable.getBooleanTopic(key);
      lastValueByKey.put(key, false);

      BooleanPublisher pub = topic.publish();
      pub.set(false);
      publishers.put(key, pub);

      ntInst.addListener(
          topic,
          EnumSet.of(NetworkTableEvent.Kind.kValueAll),
          event -> onDriveTopicEvent(key, event));
    });

    // NEW: Subscribe to HTML client command topic
    NetworkTable operatorUiTable = ntInst.getTable("OperatorUI");
    StringTopic commandTopic = operatorUiTable.getStringTopic("Command");
    
    ntInst.addListener(
        commandTopic,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> onHtmlCommandEvent(event));

    // NEW: HTTP server for HTML commands
    try {
      httpServer = HttpServer.create(new InetSocketAddress(5805), 0);
      httpServer.createContext("/command", this::handleHttpCommand);
      httpServer.setExecutor(Executors.newFixedThreadPool(2));
      httpServer.start();
      SmartLogger.logConsole("Touchscreen HTTP server started on port 5805");
    } catch (Exception e) {
      SmartLogger.logConsoleError("Failed to start HTTP server: " + e.getMessage());
    }

    SmartLogger.logConsole("Touchscreen operator interface configured", "Touchscreen Ready", 5);
  }

  private void onDriveTopicEvent(String key, NetworkTableEvent event) {
    if (event.valueData == null) {
      return;
    }

    boolean value;
    try {
      value = event.valueData.value.getBoolean();
    } catch (Exception e) {
      return;
    }

    boolean last = lastValueByKey.getOrDefault(key, false);
    lastValueByKey.put(key, value);

    if (!value || last) {
      return;
    }

    SmartDriveTarget target = targets.get(key);
    if (target == null) {
      return;
    }

    scheduleOperatorSmartDrive(key, target);
  }

  private void onHtmlCommandEvent(NetworkTableEvent event) {
    if (event.valueData == null) return;

    String pos;
    try {
      pos = event.valueData.value.getString();
    } catch (Exception e) {
      return;
    }

    if (pos == null || pos.isEmpty()) return;
    if (pos.equals(lastCommandPosition)) return;
    lastCommandPosition = pos;

    SmartDriveTarget target = targets.get(pos);
    if (target == null) {
      SmartLogger.logConsole("[Touchscreen/HTML] Unknown position: " + pos);
      return;
    }

    scheduleOperatorSmartDrive(pos, target);
  }

  private void handleHttpCommand(HttpExchange exchange) {
    try {
      if (!"POST".equals(exchange.getRequestMethod())) {
        sendHttpResponse(exchange, 405, "Method Not Allowed");
        return;
      }

      BufferedReader reader = new BufferedReader(new InputStreamReader(exchange.getRequestBody()));
      String position = reader.readLine();
      reader.close();

      if (position == null || position.isEmpty()) {
        sendHttpResponse(exchange, 400, "Missing position");
        return;
      }

      SmartDriveTarget target = targets.get(position);
      if (target == null) {
        SmartLogger.logConsole("[Touchscreen/HTTP] Unknown position: " + position);
        sendHttpResponse(exchange, 404, "Unknown position");
        return;
      }

      scheduleOperatorSmartDrive(position, target);
      sendHttpResponse(exchange, 200, "OK");
    } catch (Exception e) {
      try {
        sendHttpResponse(exchange, 500, "Error");
      } catch (Exception ignored) {}
    }
  }

  private void sendHttpResponse(HttpExchange exchange, int code, String response) throws Exception {
    // Add CORS header so browser does not block response
    exchange.getResponseHeaders().add("Access-Control-Allow-Origin", "*");
    
    byte[] bytes = response.getBytes();
    exchange.sendResponseHeaders(code, bytes.length);
    OutputStream os = exchange.getResponseBody();
    os.write(bytes);
    os.close();
  }

  private void scheduleOperatorSmartDrive(String key, SmartDriveTarget target) {
    cancelActiveOperatorDrive();

    robotState.setOperatorDriveLockout(true);

    Command cmd = SmartDriveToPosition.create(target.staging, target.precise)
        .finallyDo(interrupted -> robotState.setOperatorDriveLockout(false));

    activeOperatorDrive = cmd;
    cmd.schedule();

    SmartLogger.logConsole("[Touchscreen] SmartDrive: " + key);
  }

  public void cancelActiveOperatorDrive() {
    if (activeOperatorDrive != null) {
      activeOperatorDrive.cancel();
      activeOperatorDrive = null;
    }
    robotState.setOperatorDriveLockout(false);
    SmartLogger.logConsole("[Touchscreen] Driver override - canceled operator SmartDrive");
  }
}
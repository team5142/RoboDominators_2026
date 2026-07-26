package frc.robot.subsystems.pose;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.SmartLogger;
import java.net.HttpURLConnection;
import java.net.URL;

public class QuestNavWakeService {
  private final String questIp;
  private final int port;
  private Thread asyncThread;
  private boolean running = false;
  
  private double lastPingTime = 0.0;
  private static final double PING_INTERVAL_SEC = 2.0;

  public QuestNavWakeService(String questIp, int port) {
    this.questIp = questIp;
    this.port = port;
  }

  public void start() {
    if (running) return;
    running = true;
    asyncThread = new Thread(this::runService);
    asyncThread.setDaemon(true);
    asyncThread.setName("QuestNav-WakeService");
    asyncThread.start();
    SmartLogger.logConsole("Asynchronous Wake & Recovery Service started.", "QuestNav");
  }

  public void stop() {
    running = false;
    if (asyncThread != null) {
      asyncThread.interrupt();
    }
  }

  private void runService() {
    while (running) {
      double currentTime = Timer.getFPGATimestamp();
      if (currentTime - lastPingTime >= PING_INTERVAL_SEC) {
        lastPingTime = currentTime;
        pingAndWake();
      }
      try {
        Thread.sleep(500); // Poll sleep check at 2Hz
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        break;
      }
    }
  }

  private void pingAndWake() {
    HttpURLConnection connection = null;
    try {
      // Send a lightweight GET request to the Status/Info endpoint.
      // This wakes the USB-C Ethernet controller on the Quest from standby.
      URL url = new URL("http://" + questIp + ":" + port + "/api/info");
      connection = (HttpURLConnection) url.openConnection();
      connection.setRequestMethod("GET");
      connection.setConnectTimeout(800);
      connection.setReadTimeout(800);
      
      int responseCode = connection.getResponseCode();
      if (responseCode == 200) {
        // Success! Headset is active and webserver is running.
        // No action required.
      }
    } catch (Exception e) {
      // Connection failed (Quest is sleeping or network is down)
      // This is expected when the robot boots up on the field while the Quest is sleeping.
    } finally {
      if (connection != null) {
        connection.disconnect();
      }
    }
  }

  /**
   * Forces the headset's custom Android service to restart the QuestNav VR tracking app.
   * Useful when we see the headset is connected over NT but 'isTracking' remains false.
   */
  public void triggerAppRestart() {
    new Thread(() -> {
      HttpURLConnection connection = null;
      try {
        URL url = new URL("http://" + questIp + ":" + port + "/api/restart");
        connection = (HttpURLConnection) url.openConnection();
        connection.setRequestMethod("POST");
        connection.setConnectTimeout(1500);
        connection.setReadTimeout(1500);
        
        int responseCode = connection.getResponseCode();
        if (responseCode == 200) {
          SmartLogger.logConsole("[QuestNav] Successfully requested app tracking restart via API POST", "QuestNav");
        }
      } catch (Exception e) {
        SmartLogger.logConsoleError("Failed to send QuestNav app restart: " + e.getMessage());
      } finally {
        if (connection != null) {
          connection.disconnect();
        }
      }
    }).start();
  }
}

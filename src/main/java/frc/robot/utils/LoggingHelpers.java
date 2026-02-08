package frc.robot.utils;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Helpers for saving data logs to USB (or roboRIO storage) for use with Advantage Scope.
 * Uses WPILib's DataLogManager, which writes .wpilog files that Advantage Scope can open.
 *
 * <p>On a real robot, call {@link #startLogging()} from robotInit(); logs go to a "logs" folder
 * on the USB drive if one is attached (FAT32, &lt;= 32 GB), otherwise to /home/lvuser/logs.
 */
public final class LoggingHelpers {

  private LoggingHelpers() {}

  /** Subfolder name used on USB and on roboRIO for log files. */
  public static final String LOG_SUBDIR = "logs";

  /**
   * Preferred USB mount path on roboRIO (first USB drive). Empty means use DataLogManager default
   * (it already prefers USB when present).
   */
  public static final String USB_LOG_DIR = "/U/" + LOG_SUBDIR;

  /** Alternate USB path (some images use lowercase). */
  public static final String USB_LOG_DIR_ALT = "/u/" + LOG_SUBDIR;

  /** Flush period in seconds; tradeoff between log safety and I/O load. */
  private static final double LOG_FLUSH_PERIOD_SEC = 0.25;

  private static boolean started = false;

  /**
   * Starts the data log manager for Advantage Scope. Call once from robotInit().
   * Uses default location: USB "logs" folder if USB is present, else /home/lvuser/logs.
   */
  public static void startLogging() {
    startLoggingWithDir("");
  }

  /**
   * Starts the data log manager with an explicit directory (e.g. force USB path).
   * If dir is null or empty, uses DataLogManager default. No-op if already started.
   *
   * @param dir directory for .wpilog files (e.g. {@link #USB_LOG_DIR}), or "" for default
   */
  public static void startLoggingWithDir(String dir) {
    if (started) {
      return;
    }

    if (dir == null) {
      dir = "";
    }

    if (dir.isEmpty()) {
      DataLogManager.start();
    } else {
      DataLogManager.start(dir, "", LOG_FLUSH_PERIOD_SEC);
    }

    started = true;
  }

  /**
   * Stops the data log manager. Use only when shutting down or switching log destination.
   */
  public static void stopLogging() {
    DataLogManager.stop();
    started = false;
  }

  /**
   * Returns the managed DataLog for custom entries (e.g. custom double/string arrays for
   * Advantage Scope). Starts the manager if not already started.
   *
   * @return the WPILib DataLog instance
   */
  public static DataLog getLog() {
    return DataLogManager.getLog();
  }

  /**
   * Returns the current log directory (where .wpilog files are written).
   *
   * @return log directory path, or empty string if logging not started
   */
  public static String getLogDir() {
    return DataLogManager.getLogDir();
  }

  /**
   * Logs a message to the "messages" entry (and console). Useful for match events or debugging
   * in Advantage Scope.
   *
   * @param message message to log
   */
  public static void logMessage(String message) {
    DataLogManager.log(message);
  }

  /**
   * Enable or disable logging of NetworkTables to the log. Enabled by default; disable if you
   * only want custom log entries.
   *
   * @param enabled true to log NT, false to disable
   */
  public static void setLogNetworkTables(boolean enabled) {
    DataLogManager.logNetworkTables(enabled);
  }

  /**
   * Enable or disable logging of console output to the log.
   *
   * @param enabled true to log console, false to disable
   */
  public static void setLogConsoleOutput(boolean enabled) {
    DataLogManager.logConsoleOutput(enabled);
  }
}

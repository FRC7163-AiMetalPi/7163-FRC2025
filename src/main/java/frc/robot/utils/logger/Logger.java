package frc.robot.utils.logger;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Optional;
import java.util.concurrent.locks.ReentrantLock;

import frc.robot.constants.FileConstants;
import frc.robot.constants.LoggerConstants;

public class Logger {

  private ArrayList<double[]> cache = new ArrayList<double[]>();
  private final ReentrantLock cacheLock = new ReentrantLock();

  private final long creationTime = Calendar.getInstance().getTime().toInstant().toEpochMilli();

  private Thread thread;

  private static boolean pausedGlobal = false;
  private boolean pausedLocal = false;
  private static boolean stopped = false;

  private File file;
  private static Optional<String> pathName = Optional.empty();
  private String fileName;

  public Logger(String fileName, String[] columns) {
    
    if(pathName.isEmpty() && !stopped)
      pathName = findSavePath();
    if (!stopped && !pathName.isEmpty())
      this.fileName = pathName.get() + "//" + fileName;
    else
      stopped = true;
    makeFile();
    
    if(!stopped){
      save(columns);

      launchThread();
    }
  }


  public void log(double[] values) {

    var row = new double[values.length + 1];
    row[0] = (double)(Calendar.getInstance().getTime().toInstant().toEpochMilli() - creationTime) / 1000d;
    for (int i = 0; i < values.length; i++) {
      row[i+1] = values[i];
    }

    cacheLock.lock();
    cache.add(row);
    cacheLock.unlock();
  }

  private Optional<String> findSavePath() {
    if (pausedGlobal || pausedLocal || stopped) {
      return Optional.empty();
    }
    String pathDrive = "";
                                                                                                    // try to find a drive
    try {
      for (String tmpDrivePath : FileConstants.PATH_USB) {
        if (new File(tmpDrivePath).exists()) {
          pathDrive = tmpDrivePath;
          break;
        }
      }
    } catch (SecurityException e) {
      stopped = true;
      System.out.println("[" + fileName + " Logger] Log Drive Path Security Exception : Killing Logger");
    }
                                                                                                    // try to create a folder
    try {
      if (pathDrive != ""){
        for (int i = 0; i < LoggerConstants.REPEAT_LIMIT_LOGGER_FOLDER_CREATION; i++){
          if (!new File(pathDrive+"LogFile_("+i+")").exists()) {
            File folder = new File(pathDrive+"LogFile_("+i+")");
            folder.mkdir();
            return Optional.of(pathDrive+"LogFile_("+i+")");
          }
        }
      }
    } catch (SecurityException e) {
      stopped = true;
      System.out.println("[" + fileName + " Logger] Log Folder Path Security Exception : Killing Logger");
    }
    System.out.println("[" + fileName + " Logger] Log Save Path Not Found : Killing Logger");
    stopped = true;
    return Optional.empty();
  }

  private void makeFile() {
    if (pausedGlobal || pausedLocal || stopped) {
      return;
    }

    file = new File(fileName + ".csv");
    if (file == null) {
      stopped = true;
      return;
    }

    try {
      String filePath = "";
      for (int i = 0; !file.createNewFile(); i++) {
        if (i > LoggerConstants.REPEAT_LIMIT_LOGGER_CREATION) {
          stopped = true;
          System.out.println("[" + fileName + " Logger] File Creation Attempt Limit Exceeded : Killing Logger");
          return;
        }
        filePath = fileName + "_(" + i + ").csv";
        file = new File(filePath);
      }
      
      if (!stopped) {
        System.out.println("[" + fileName + " Logger] Log File Initialised : " + filePath);
      }

    } catch (IOException e) {
      stopped = true;
      System.out.println("[" + fileName + " Logger] File Creation Failed : IOException : " + e + " : Killing Logger");
    } catch (SecurityException e) {
      stopped = true;
      System.out.println("[" + fileName + " Logger] File Creation Failed : Security Exception : " + e + " : Killing Logger");
    }

  }

  public void save() {
    save(new String[0]);
  }

  public void save(String[] override) {
    cacheLock.lock();

    try {
      FileWriter writer = new FileWriter(file, true);

      if (override.length != 0) {
        writer.write("Time");
        for (int i = 0; i < override.length; i++) {
          writer.write(", " + override[i]);
        }
        writer.write(System.lineSeparator());
      }

      while (!cache.isEmpty()) {
        
        var row = cache.remove(0);
        
        writer.write(String.valueOf(row[0])); // timestamp
        for (int i = 1; i < row.length; i++) {
          writer.write(", " + row[i]);
        }
        
        writer.write(System.lineSeparator());
      }
      writer.close();
    } catch (IOException e) {
      System.out.println("[" + fileName + " Logger] File Save Failed : IOExeption : " + e);
      stopped = true;
    } catch (SecurityException e) {
      System.out.println("[" + fileName + " Logger] File Save Failed : Security Exeption : " + e);
      stopped = true;
    } finally {
      if (!cache.isEmpty()) {
        System.out.println("[" + fileName + " Logger] Save Error : Killing Logger");
      }
    }
    cacheLock.unlock();
  }

  /** reversibly pauses operation of all loggers (will not effect per logger pauses)*/
  public static void pauseAllLoggers() {
    if (!stopped) {
      pausedGlobal = true;
    }
  }

  /** restarts all loggers when paused (will not effect per logger pauses) */
  public static void unpauseAllLoggers() {
    if (!stopped) {
      pausedGlobal = false;
    }
  }

  /** reversibly pauses operation of instance loggers (will not effect global logger pauses)*/
  public void pauseLogger() {
    if (!stopped) {
      pausedLocal = true;
    }
  }

  /** restarts instance loggers when paused (will not effect global logger pauses) */
  public void unpauseLogger() {
    if (!stopped) {
      pausedLocal = false;
    }
  }

  /** irreversably kills all loggers */
  public static void stop() {
    if (!stopped) {
      pauseAllLoggers();
      stopped = true;
    }
  }

  private void launchThread() {
    if (stopped) {
      return;
    }
    thread = new Thread(() -> {
      try {
        while (!stopped) {
          if (!cache.isEmpty() && !pausedGlobal && !pausedLocal && !stopped) {
            save();
          }

          Thread.sleep(1000 / LoggerConstants.SAVE_RATE);
        }
        cache.clear();
      } catch (InterruptedException e) {
        System.out.println("[" + fileName + " Logger] Save Thread Interrupted : " + e);
      }
    });
    thread.start();
  }
}

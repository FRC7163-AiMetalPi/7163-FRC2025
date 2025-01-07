package frc.robot.utils.logger;

import java.time.Instant;
import java.util.ArrayList;
import java.util.Calendar;

public class LogRow {

  private Instant timestamp;
  private ArrayList<String> lines;


  public LogRow() {
    timestamp = Calendar.getInstance().getTime().toInstant();
    lines = new ArrayList<String>();
  }

  public void addLine(String line) {
    lines.add(line);
  }

  public Instant getTime() {
    return timestamp;
  }

  public String[] getLines() {
    return lines.toArray(new String[0]);
  }

}

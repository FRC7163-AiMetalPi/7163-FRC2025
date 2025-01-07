package frc.robot.shufflecontrol;

import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

public class ShuffleTabController {
  private final ShuffleboardTab tab;
  private final HashMap<String, SimpleWidget> entries = new HashMap<String, SimpleWidget>();

  /* name : quantity registered */
  private static final HashMap<String, Integer> widgetTitles = new HashMap<String, Integer>();
  /* name : quantity registered */
  private static final HashMap<String, Integer> tabTitles = new HashMap<String, Integer>();

  public ShuffleTabController(String title) {
    tab = Shuffleboard.getTab(getUniqueTabTitle(title));
    System.out.println(tab.hashCode());
  }

  public SimpleWidget createWidget(String title) {
    var entry = entries.get(title);
    if (entry != null)
      return entry;

    var widget = tab.add(getUniqueWidgetTitle(title), 0);
    entries.put(title, widget);
    return widget;
  }

  public SimpleWidget createWidget(String title, WidgetType type, int coloumn, int row) {
    return createWidget(title).withWidget(type).withPosition(coloumn, row).withSize(1, 1);
  }

  public SimpleWidget createWidget(String title, WidgetType type, int coloumn, int row, int width, int height) {
    return createWidget(title, type, coloumn, row).withSize(width, height);
  }

  public SimpleWidget getWidget(String title) {
    var entry = entries.get(title);
    if (entry != null) {
      return entry;
    }
    
    System.out.println("Widget " + title + " is not registered");
    return null;
  }

  public GenericEntry getEntry(String title) {
    return getWidget(title).getEntry();
  }

  private static String getUniqueWidgetTitle(String title) {
    var count = widgetTitles.get(title);
    if (count == null) {
      widgetTitles.put(title, 1);
      return title;
    }

    widgetTitles.put(title, count + 1);
    return title + "(" + Integer.toString(count + 1) + ")";
  }

  private static String getUniqueTabTitle(String title) {
    var count = tabTitles.get(title);
    if (count == null) {
      tabTitles.put(title, 1);
      return title;
    }

    tabTitles.put(title, count + 1);
    return title + "(" + Integer.toString(count + 1) + ")";
  }
}

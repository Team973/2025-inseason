package com.team973.frc2025.shared;

import edu.wpi.first.wpilibj.RobotBase;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.UUID;

/** Tracks start-up and caught crash events, logging them to a file which doesn't roll over */
public class CrashTracker {
  private static boolean m_exceptionHappened = false;

  private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();

  public static synchronized boolean getExceptionHappened() {
    return m_exceptionHappened;
  }

  public static synchronized void resetExceptionHappened() {
    m_exceptionHappened = false;
  }

  public static synchronized void logException(String context, Throwable e) {
    m_exceptionHappened = true;
    logMarker(context, e);
  }

  private static void logMarker(String mark, Throwable e) {
    System.out.printf("Exception at mark %s\n", mark);
    System.out.println(e);

    if (RobotBase.isSimulation()) {
      return;
    }

    try (PrintWriter writer =
        new PrintWriter(new FileWriter("/home/lvuser/exception_log.txt", true))) {
      writer.print(RUN_INSTANCE_UUID.toString());
      writer.print(", ");
      writer.print(mark);
      writer.print(", ");
      writer.print(new Date().toString());

      if (e != null) {
        writer.print(", ");
        e.printStackTrace(writer);
      }

      writer.println();
    } catch (IOException ie) {
      ie.printStackTrace();
    }
  }
}

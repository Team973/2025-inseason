// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team973.analysis;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.io.IOException;
import java.time.format.DateTimeFormatter;
import java.util.HashMap;
import java.util.InputMismatchException;
import java.util.Map;

public final class MotorDraw {
  private static final DateTimeFormatter m_timeFormatter =
      DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss");

  /** Main function. */
  public static void main(String[] args) {
    if (args.length != 1) {
      System.err.println("Usage: printlog <file>");
      System.exit(1);
      return;
    }
    DataLogReader reader;
    try {
      reader = new DataLogReader(args[0]);
    } catch (IOException ex) {
      System.err.println("could not open file: " + ex.getMessage());
      System.exit(1);
      return;
    }
    if (!reader.isValid()) {
      System.err.println("not a log file");
      System.exit(1);
      return;
    }

    Map<Integer, DataLogRecord.StartRecordData> entries = new HashMap<>();
    for (DataLogRecord record : reader) {
      if (record.isStart()) {
        try {
          DataLogRecord.StartRecordData data = record.getStartData();
          System.out.println(
              "Start("
                  + data.entry
                  + ", name='"
                  + data.name
                  + "', type='"
                  + data.type
                  + "', metadata='"
                  + data.metadata
                  + "') ["
                  + (record.getTimestamp() / 1000000.0)
                  + "]");
          if (entries.containsKey(data.entry)) {
            System.out.println("...DUPLICATE entry ID, overriding");
          }
          entries.put(data.entry, data);
        } catch (InputMismatchException ex) {
          System.out.println("Start(INVALID)");
        }
      } else if (!record.isControl()) {
        DataLogRecord.StartRecordData entry = entries.get(record.getEntry());
        if (entry == null) {
          continue;
        }
        if (entry.name.equals("/Robot/robot/claw/conveyorMotor/Voltage")) {
          System.out.printf(
              "Conveyor took %fv at %fs\n", record.getDouble(), record.getTimestamp() / 1000000.0);
        }
      }
    }
  }

  private MotorDraw() {}
}

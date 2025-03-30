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
    doSummaryForMotor(args[0], "/Robot/robot/claw/clawMotor/");
    doSummaryForMotor(args[0], "/Robot/robot/wrist/wristMotor/");
    doSummaryForMotor(args[0], "/Robot/robot/Arm/armMotor/");
    doSummaryForMotor(args[0], "/Robot/robot/elevator/motorLeft/");
    doSummaryForMotor(args[0], "/Robot/robot/elevator/motorRight/");
    doSummaryForMotor(args[0], "/Robot/robot/climb manager/climb motor/");
    doSummaryForMotor(args[0], "/Robot/robot/drive/swerve/mod0/Drive Motor/");
    doSummaryForMotor(args[0], "/Robot/robot/drive/swerve/mod0/Angle Motor/");
    doSummaryForMotor(args[0], "/Robot/robot/drive/swerve/mod1/Drive Motor/");
    doSummaryForMotor(args[0], "/Robot/robot/drive/swerve/mod1/Angle Motor/");
    doSummaryForMotor(args[0], "/Robot/robot/drive/swerve/mod2/Drive Motor/");
    doSummaryForMotor(args[0], "/Robot/robot/drive/swerve/mod2/Angle Motor/");
    doSummaryForMotor(args[0], "/Robot/robot/drive/swerve/mod3/Drive Motor/");
    doSummaryForMotor(args[0], "/Robot/robot/drive/swerve/mod3/Angle Motor/");
  }

  private static final void doSummaryForMotor(String logPath, String loggedMotor) {
    DataLogReader reader;
    try {
      reader = new DataLogReader(logPath);
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
    double VoltageSum = 0;
    double StatorSum = 0;
    double WattageSum = 0;
    double PreviousVoltage = 0;
    double PreviousStator = 0;
    double PreviousVoltageTimestamp = 0;
    double PreviousStatorTimestamp = 0;

    for (DataLogRecord record : reader) {
      if (record.isStart()) {
        try {
          DataLogRecord.StartRecordData data = record.getStartData();
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
        // **Logged motor measurements**
        if (entry.name.equals(loggedMotor + "Voltage")) {
          double timeDelta = 0.02;
          VoltageSum = VoltageSum + Math.abs(record.getDouble()) * timeDelta;
          PreviousVoltage = record.getDouble();
          PreviousVoltageTimestamp = record.getTimestamp();
        }
        if (entry.name.equals(loggedMotor + "Stator Current")) {
          double timeDelta = 0.02;
          StatorSum = StatorSum + record.getDouble() * timeDelta;
          PreviousStator = record.getDouble();
          PreviousStatorTimestamp = record.getTimestamp();
          // **Recieves latest Stator and Voltage values, multiplies and adds them to overall
          // wattage sum.**
          double currentWattage = PreviousVoltage * PreviousStator;
          WattageSum = WattageSum + currentWattage * timeDelta;
        }
      }
    }
    // **Printing out sums of all measurements**
    System.out.printf(loggedMotor + " stator current took %fa \n", StatorSum / 150);
    System.out.printf(loggedMotor + " motor voltage took %fv \n", VoltageSum / 150);
    System.out.printf(loggedMotor + " motor wattage took %fw \n", WattageSum / 150);
  }

  private MotorDraw() {}
}

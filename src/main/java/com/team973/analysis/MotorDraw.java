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
    doSummaryForMotor(
        args[0],
        "/Robot/robot/claw/conveyorMotor/",
        "/Robot/robot/claw/clawMotor/",
        "/Robot/robot/wrist/wristMotor/",
        "/Robot/robot/Arm/armMotor/", "/Robot/robot/elevator/motorLeft/", "/Robot/robot/elevator/motorRight", "/Robot/robot/climb manager/climb motor/");
  }

  private static final void doSummaryForMotor(
      String logPath, String conveyorMotor, String clawMotor, String wristMotor, String armMotor, String elevatorLeftMotor, String elevatorRightMotor, String climbMotor, String frontLeftDriveMotor, String frontLeftSteerMotor, String frontRightDriveMotor, String frontRightSteerMotor, String backLeftDriveMotor, String backLeftSteerMotor, String backRightDriveMotor, String backRightSteerMotor) {
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
    double conveyorVoltageSum = 0;
    double conveyorStatorSum = 0;
    double conveyorWattageSum = 0;
    double conveyorPreviousVoltage = 0;
    double conveyorPreviousStator = 0;
    double conveyorPreviousVoltageTimestamp = 0;
    double conveyorPreviousStatorTimestamp = 0;
    double wristVoltageSum = 0;
    double wristStatorSum = 0;
    double wristWattageSum = 0;
    double wristPreviousVoltage = 0;
    double wristPreviousStator = 0;
    double wristPreviousVoltageTimestamp = 0;
    double wristPreviousStatorTimestamp = 0;
    double clawVoltageSum = 0;
    double clawStatorSum = 0;
    double clawWattageSum = 0;
    double clawPreviousVoltage = 0;
    double clawPreviousStator = 0;
    double clawPreviousVoltageTimestamp = 0;
    double clawPreviousStatorTimestamp = 0;
    double armVoltageSum = 0;
    double armStatorSum = 0;
    double armWattageSum = 0;
    double armPreviousVoltage = 0;
    double armPreviousStator = 0;
    double armPreviousVoltageTimestamp = 0;
    double armPreviousStatorTimestamp = 0;
    double elevatorLeftVoltageSum = 0;
    double elevatorLeftStatorSum = 0;
    double elevatorLeftWattageSum = 0;
    double elevatorLeftPreviousVoltage = 0;
    double elevatorLeftPreviousStator = 0;
    double elevatorLeftPreviousVoltageTimestamp = 0;
    double elevatorLeftPreviousStatorTimestamp = 0;
    double elevatorRightVoltageSum = 0;
    double elevatorRightStatorSum = 0;
    double elevatorRightWattageSum = 0;
    double elevatorRightPreviousVoltage = 0;
    double elevatorRightPreviousStator = 0;
    double elevatorRightPreviousVoltageTimestamp = 0;
    double elevatorRightPreviousStatorTimestamp = 0;
    double climbVoltageSum = 0;
    double climbStatorSum = 0;
    double climbWattageSum = 0;
    double climbPreviousVoltage = 0;
    double climbPreviousStator = 0;
    double climbPreviousVoltageTimestamp = 0;
    double climbPreviousStatorTimestamp = 0;
    double frontLeftDriveVoltageSum = 0;
    double frontLeftDriveStatorSum = 0;
    double frontLeftDriveWattageSum = 0;
    double frontLeftDrivePreviousVoltage = 0;
    double frontLeftDrivePreviousStator = 0;
    double frontLeftDrivePreviousVoltageTimestamp = 0;
    double frontLeftDrivePreviousStatorTimestamp = 0;
    double frontLeftSteerVoltageSum = 0;
    double frontLeftSteerStatorSum = 0;
    double frontLeftSteerWattageSum = 0;
    double frontLeftSteerPreviousVoltage = 0;
    double frontLeftSteerPreviousStator = 0;
    double frontLeftSteerPreviousVoltageTimestamp = 0;
    double frontLeftSteerPreviousStatorTimestamp = 0;
    double frontRightDriveVoltageSum = 0;
    double frontRightDriveStatorSum = 0;
    double frontRightDriveWattageSum = 0;
    double frontRightDrivePreviousVoltage = 0;
    double frontRightDrivePreviousStator = 0;
    double frontRightDrivePreviousVoltageTimestamp = 0;
    double frontRightDrivePreviousStatorTimestamp = 0;
    double frontRightSteerVoltageSum = 0;
    double frontRightSteerStatorSum = 0;
    double frontRightSteerWattageSum = 0;
    double frontRightSteerPreviousVoltage = 0;
    double frontRightSteerPreviousStator = 0;
    double frontRightSteerPreviousVoltageTimestamp = 0;
    double frontRightSteerPreviousStatorTimestamp = 0;
    double backLeftDriveVoltageSum = 0;
    double backLeftDriveStatorSum = 0;
    double backLeftDriveWattageSum = 0;
    double backLeftDrivePreviousVoltage = 0;
    double backLeftDrivePreviousStator = 0;
    double backLeftDrivePreviousVoltageTimestamp = 0;
    double backLeftDrivePreviousStatorTimestamp = 0;
    double backLeftSteerVoltageSum = 0;
    double backLeftSteerStatorSum = 0;
    double backLeftSteerWattageSum = 0;
    double backLeftSteerPreviousVoltage = 0;
    double backLeftSteerPreviousStator = 0;
    double backLeftSteerPreviousVoltageTimestamp = 0;
    double backLeftSteerPreviousStatorTimestamp = 0;
    double backRightDriveVoltageSum = 0;
    double backRightDriveStatorSum = 0;
    double backRightDriveWattageSum = 0;
    double backRightDrivePreviousVoltage = 0;
    double backRightDrivePreviousStator = 0;
    double backRightDrivePreviousVoltageTimestamp = 0;
    double backRightDrivePreviousStatorTimestamp = 0;
    double backRightSteerVoltageSum = 0;
    double backRightSteerStatorSum = 0;
    double backRightSteerWattageSum = 0;
    double backRightSteerPreviousVoltage = 0;
    double backRightSteerPreviousStator = 0;
    double backRightSteerPreviousVoltageTimestamp = 0;
    double backRightSteerPreviousStatorTimestamp = 0;
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
        // **Conveyor motor measurements**
        if (entry.name.equals(conveyorMotor + "Voltage")) {
          conveyorVoltageSum = conveyorVoltageSum + record.getDouble();
          conveyorPreviousVoltage = record.getDouble();
          conveyorPreviousVoltageTimestamp = record.getTimestamp();
        }
        if (entry.name.equals(conveyorMotor + "Stator Current")) {
          conveyorStatorSum = (conveyorStatorSum + record.getDouble()) * 0.02;
          conveyorPreviousStator = record.getDouble();
          conveyorPreviousStatorTimestamp = record.getTimestamp();
          // **Recieves latest Stator and Voltage values, multiplies and adds them to overall
          // wattage sum.**
          conveyorWattageSum =
              conveyorPreviousVoltage * conveyorPreviousStator + conveyorWattageSum;
        }
        // **Wrist motor measurements**
        if (entry.name.equals(wristMotor + "Voltage")) {
          wristVoltageSum = conveyorVoltageSum + record.getDouble();
          wristPreviousVoltage = record.getDouble();
          wristPreviousVoltageTimestamp = record.getTimestamp();
        }
        if (entry.name.equals(wristMotor + "Stator Current")) {
          wristStatorSum = (wristStatorSum + record.getDouble()) * 0.02;
          wristPreviousStator = record.getDouble();
          wristPreviousStatorTimestamp = record.getTimestamp();
          // **Recieves latest Stator and Voltage values, multiplies and adds them to overall
          // wattage sum.**
          wristWattageSum = wristPreviousVoltage * wristPreviousStator + wristWattageSum;
        }
        // **Claw motor measurements**
        if (entry.name.equals(clawMotor + "Voltage")) {
          clawVoltageSum = clawVoltageSum + record.getDouble();
          clawPreviousVoltage = record.getDouble();
          clawPreviousVoltageTimestamp = record.getDouble();
        }
        if (entry.name.equals(clawMotor + "Stator Current")) {
          clawStatorSum = (clawStatorSum + record.getDouble()) * 0.02;
          clawPreviousStator = record.getDouble();
          clawPreviousStatorTimestamp = record.getTimestamp();
          // **Recieves latest Stator and Voltage values, multiplies and adds them to overall
          // wattage sum.**
          clawWattageSum = clawPreviousVoltage * clawPreviousStator + clawWattageSum;
        }
        // **Arm motor measurements**
        if (entry.name.equals(armMotor + "Voltage")) {
          armVoltageSum = armVoltageSum + record.getDouble();
          armPreviousVoltage = record.getDouble();
          armPreviousVoltageTimestamp = record.getDouble();
        }
        if (entry.name.equals(armMotor + "Stator Current")) {
          armStatorSum = (armStatorSum + record.getDouble()) * 0.02;
          armPreviousStator = record.getDouble();
          armPreviousStatorTimestamp = record.getTimestamp();
          // **Recieves latest Stator and Voltage values, multiplies and adds them to overall
          // wattage sum.**
          armWattageSum = armPreviousVoltage * armPreviousStator + armWattageSum;
        }
      }
    }
    // **Printing out sums of all measurements**
    System.out.printf(conveyorMotor + " stator current took %fa \n", conveyorStatorSum);
    System.out.printf(conveyorMotor + " motor voltage took %fv \n", conveyorVoltageSum);
    System.out.printf(conveyorMotor + " motor wattage took %fw \n", conveyorWattageSum);
    System.out.printf(clawMotor + " stator current took %fa \n", clawStatorSum);
    System.out.printf(clawMotor + " motor voltage took %fv \n", clawVoltageSum);
    System.out.printf(clawMotor + " motor wattage took %fw \n", clawWattageSum);
    System.out.printf(armMotor + " stator current took %fa \n", armStatorSum);
    System.out.printf(armMotor + " motor voltage took %fv \n", armVoltageSum);
    System.out.printf(armMotor + " motor wattage took %fw \n", armWattageSum);
    System.out.printf(wristMotor + " stator current took %fa \n", wristStatorSum);
    System.out.printf(wristMotor + " motor voltage took %fv \n", wristVoltageSum);
    System.out.printf(wristMotor + " motor wattage took %fw \n", wristWattageSum);
  }

  private MotorDraw() {}
}

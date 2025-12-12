package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.lib.util.Conversions;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;

public class ElevatorSim extends Elevator {
  private final edu.wpi.first.wpilibj.simulation.ElevatorSim m_sim;
  private final TalonFXSimState m_motorRightSimState;

  private final double m_motorRotToElevatorHeightMeters;
  private double m_lastPoseMeters;

  public ElevatorSim(Logger logger, CANdleManger candle, RobotInfo robotInfo) {
    super(logger, candle, robotInfo);

    m_motorRotToElevatorHeightMeters =
        robotInfo.ELEVATOR_INFO.MOTOR_ROT_TO_HEIGHT_INCHES * Conversions.Distance.METERS_PER_INCH;

    m_sim =
        new edu.wpi.first.wpilibj.simulation.ElevatorSim(
            DCMotor.getKrakenX60(2),
            1.0 / robotInfo.ELEVATOR_INFO.MOTOR_GEAR_RATIO,
            5.0,
            0.029,
            0.0,
            Conversions.Distance.METERS_PER_INCH * 30.0,
            true,
            0.0);

    m_motorRightSimState = m_motorRight.getSimState();
    m_lastPoseMeters = m_sim.getPositionMeters();
  }

  public Pose3d getPose() {
    return new Pose3d(0, 0, m_sim.getPositionMeters(), new Rotation3d());
  }

  public void syncSensors() {
    m_motorRightSimState.addRotorPosition(
        (m_sim.getPositionMeters() - m_lastPoseMeters) / m_motorRotToElevatorHeightMeters);
    m_lastPoseMeters = m_sim.getPositionMeters();

    super.syncSensors();
  }

  public void update() {
    super.update();

    m_sim.setInputVoltage(m_motorRightSimState.getMotorVoltage());

    m_sim.update(0.02);
  }
}

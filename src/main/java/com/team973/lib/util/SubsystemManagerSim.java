package com.team973.lib.util;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.CANdleManger;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.ElevatorIO;
import com.team973.frc2025.subsystems.ElevatorSim;
import com.team973.frc2025.subsystems.swerve.SwerveModuleSim;
import com.team973.lib.devices.GreyPigeon;
import com.team973.lib.devices.GreyPigeonSim;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class SubsystemManagerSim extends SubsystemManager {
  private final SwerveDriveSimulation m_swerveDriveSimulation;
  private final GreyPigeon m_pigeon;

  private Supplier<Pose3d> m_elevatorPoseSupplier;

  public SubsystemManagerSim(RobotInfo robotInfo, Logger logger) {
    super(robotInfo, logger);

    m_swerveDriveSimulation =
        new SwerveDriveSimulation(
            robotInfo.DRIVE_INFO.getDriveTrainSimulationConfig(),
            robotInfo.DRIVE_INFO.getSimStartingPose());

    SimulatedArena.getInstance().addDriveTrainSimulation(m_swerveDriveSimulation);

    m_pigeon =
        new GreyPigeonSim(logger.subLogger("pigeon"), m_swerveDriveSimulation.getGyroSimulation());
  }

  public DriveController initDriveController(
      Logger logger, AtomicBoolean readyToScore, AtomicBoolean readyToBackOff) {
    return new DriveController(
        logger,
        readyToScore,
        readyToBackOff,
        new SwerveModuleSim(
            0,
            m_swerveDriveSimulation.getModules()[0],
            getRobotInfo().DRIVE_INFO.FRONT_LEFT_CONSTANTS,
            logger.subLogger("swerve/mod0")),
        new SwerveModuleSim(
            1,
            m_swerveDriveSimulation.getModules()[1],
            getRobotInfo().DRIVE_INFO.FRONT_RIGHT_CONSTANTS,
            logger.subLogger("swerve/mod1")),
        new SwerveModuleSim(
            2,
            m_swerveDriveSimulation.getModules()[2],
            getRobotInfo().DRIVE_INFO.BACK_LEFT_CONSTANTS,
            logger.subLogger("swerve/mod2")),
        new SwerveModuleSim(
            3,
            m_swerveDriveSimulation.getModules()[3],
            getRobotInfo().DRIVE_INFO.BACK_RIGHT_CONSTANTS,
            logger.subLogger("swerve/mod3")),
        m_pigeon);
  }

  public ElevatorIO initElevator(Logger logger, CANdleManger candle, RobotInfo robotInfo) {
    ElevatorSim elevator = new ElevatorSim(logger, candle, robotInfo);

    m_elevatorPoseSupplier = () -> elevator.getPose();

    return elevator;
  }

  public void simulationUpdate() {
    m_pigeon.simulationUpdate();

    getLogger().log("robotTruthPose", m_swerveDriveSimulation.getSimulatedDriveTrainPose());
    getLogger().log("components", new Pose3d[] {m_elevatorPoseSupplier.get()});
  }
}

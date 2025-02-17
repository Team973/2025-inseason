package com.team973.frc2025.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.shared.RobotInfo.DriveInfo;
import com.team973.lib.devices.GreyPigeon;
import com.team973.lib.util.Logger;
import com.team973.lib.util.PerfLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;

public class OdometrySupplier {

  public interface OdometryReceiver {
    public void observeOdometryData(
        double sampleTime, Rotation2d yawDegrees, SwerveModulePosition[] modulePositions);
  }

  private List<OdometryReceiver> m_receivers = new ArrayList<OdometryReceiver>();

  private final BaseStatusSignal[] m_allStatusSignals;
  private final StatusSignal<Angle> m_yawGetter;
  private final StatusSignal<AngularVelocity> m_angularVelocity;

  private final SwerveDriveOdometry m_swerveOdometry;
  private final SwerveModule[] m_swerveModules;
  private final GreyPigeon m_pigeon;
  private PerfLogger m_loopPeriodTracker;
  private PerfLogger m_loopDurationTracker;
  private final Thread m_thread;
  private Pose2d m_lastPoseMeters;

  private boolean m_started = false;

  private Logger m_logger;
  private Logger m_perfLogger;

  public OdometrySupplier(GreyPigeon pigeon, SwerveModule[] swerveModules, Logger logger) {
    m_thread = new Thread(this::run);
    m_thread.setName("swerve.OdometryPoseSupplier");
    m_thread.setDaemon(false);

    m_pigeon = pigeon;
    m_swerveModules = swerveModules;
    m_logger = logger;
    m_perfLogger = logger.subLogger("perf", 0.25);
    m_loopPeriodTracker = new PerfLogger(m_perfLogger.subLogger("period"));
    m_loopDurationTracker = new PerfLogger(m_perfLogger.subLogger("duration"));

    List<StatusSignal<Angle>> angleSignals = m_pigeon.getAngleStatusSignals();
    List<StatusSignal<AngularVelocity>> angularVelocitySignals =
        m_pigeon.getAngularVelocityStatusSignals();
    m_yawGetter = angleSignals.get(0);
    m_angularVelocity = angularVelocitySignals.get(0);
    m_allStatusSignals =
        new BaseStatusSignal
            [4 * m_swerveModules.length + angleSignals.size() + angularVelocitySignals.size()];

    int i = 0;
    for (SwerveModule mod : m_swerveModules) {
      for (BaseStatusSignal s : mod.allStatusSignals()) {
        m_allStatusSignals[i++] = s;
      }
    }
    m_allStatusSignals[i++] = m_yawGetter;
    m_allStatusSignals[i++] = m_angularVelocity;

    BaseStatusSignal.setUpdateFrequencyForAll(
        RobotInfo.DriveInfo.STATUS_SIGNAL_FREQUENCY, m_allStatusSignals);

    BaseStatusSignal.waitForAll(1.0, m_allStatusSignals);
    Measure<AngleUnit> yawDegrees =
        BaseStatusSignal.getLatencyCompensatedValue(m_yawGetter, m_angularVelocity);
    m_swerveOdometry =
        new SwerveDriveOdometry(
            DriveInfo.SWERVE_KINEMATICS,
            Rotation2d.fromDegrees(yawDegrees.magnitude()),
            getPositions());
    m_lastPoseMeters = getPoseMeters();
  }

  public void addReceiver(OdometryReceiver newReceiver) {
    m_receivers.add(newReceiver);
  }

  public void start() {
    if (!m_started) {
      m_thread.start();
    }
    m_started = true;
  }

  public Pose2d getPoseMeters() {
    return m_swerveOdometry.getPoseMeters();
  }

  public Translation2d getVelocityMetersPerSeconds() {
    return m_lastPoseMeters
        .minus(getPoseMeters())
        .getTranslation()
        .times(1.0 / RobotInfo.DriveInfo.STATUS_SIGNAL_FREQUENCY);
  }

  public synchronized void resetPosition(Pose2d pose) {
    m_swerveOdometry.resetPosition(m_pigeon.getYaw(), getPositions(), pose);
    log();
  }

  private int m_successfulCycles = 0;
  private int m_failedCycles = 0;

  public void run() {
    try {
      doRun();
    } catch (Exception ex) {
      System.out.printf(
          "OdometrySupplier thread died.\n%s\n%s\n", ex.getMessage(), ex.getStackTrace());
      ex.printStackTrace();
      DriverStation.reportError(ex.getMessage(), ex.getStackTrace());
    }
  }

  public void doRun() {
    // Ref
    // https://api.ctr-electronics.com/phoenix6/release/java/src-html/com/ctre/phoenix6/mechanisms/swerve/SwerveDrivetrain.html#line.184
    Threads.setCurrentThreadPriority(true, 1);
    double lastCycleTimestamp = Timer.getFPGATimestamp();
    while (true) {
      // Since we've set the update frequency, we will never actually wait
      // this long for new signal. We expect to get a signal in at most 1/freq
      StatusCode status =
          BaseStatusSignal.waitForAll(
              2.0 / RobotInfo.DriveInfo.STATUS_SIGNAL_FREQUENCY, m_allStatusSignals);

      double runStartedAt = Timer.getFPGATimestamp();
      doCycle();

      synchronized (this) {
        if (status.isOK()) {
          m_successfulCycles++;
        } else {
          m_failedCycles++;
        }

        double runFinishedAt = Timer.getFPGATimestamp();
        m_loopPeriodTracker.observe(runFinishedAt - lastCycleTimestamp);
        m_loopDurationTracker.observe(runFinishedAt - runStartedAt);
        lastCycleTimestamp = runFinishedAt;
      }
    }
  }

  private void doCycle() {
    double timestamp = Timer.getFPGATimestamp();
    Rotation2d yawDegrees = m_pigeon.getYaw();
    SwerveModulePosition[] modulePositions = getPositions();

    for (OdometryReceiver receiver : m_receivers) {
      receiver.observeOdometryData(timestamp, yawDegrees, modulePositions);
    }
  }

  public void log() {
    m_perfLogger.log("thread/successful cycles", () -> getSuccessfulCycles());
    m_perfLogger.log("thread/failed cycles", () -> getFailedCycles());
  }

  public SwerveModulePosition[] getPositions() {
    var positions = new SwerveModulePosition[4];
    for (var mod : m_swerveModules) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public synchronized int getSuccessfulCycles() {
    return m_successfulCycles;
  }

  public synchronized int getFailedCycles() {
    return m_failedCycles;
  }
}

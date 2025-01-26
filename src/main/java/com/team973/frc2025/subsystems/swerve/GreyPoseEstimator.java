package com.team973.frc2025.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.google.common.collect.ImmutableList;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.shared.RobotInfo.DriveInfo;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.lib.devices.GreyPigeon;
import com.team973.lib.util.SignalAggregator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

public class GreyPoseEstimator {

  private final BaseStatusSignal[] m_allStatusSignals;
  private final StatusSignal<Angle> m_yawGetter;
  private final StatusSignal<AngularVelocity> m_angularVelocity;

  private final SwerveDriveOdometry m_swerveOdometry;
  private final SwerveModule[] m_swerveModules;
  private final GreyPigeon m_pigeon;
  private SignalAggregator m_loopTimeTracker = new SignalAggregator();
  private final Thread m_thread;
  private Pose2d m_lastPoseMeters;

  private DriveController m_driveController;

  public GreyPoseEstimator(
      GreyPigeon pigeon, SwerveModule[] swerveModules, DriveController m_DriveController) {
    m_thread = new Thread(this::run);
    m_thread.setName("swerve.GreyPostEstimator");
    m_thread.setDaemon(false);

    m_pigeon = pigeon;
    m_swerveModules = swerveModules;
    m_driveController = m_DriveController;

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

  public void start() {
    m_thread.start();
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
    // Ref
    // https://api.ctr-electronics.com/phoenix6/release/java/src-html/com/ctre/phoenix6/mechanisms/swerve/SwerveDrivetrain.html#line.184
    Threads.setCurrentThreadPriority(true, 1);
    double runStartedAt = Timer.getFPGATimestamp();
    while (true) {
      // Since we've set the update frequency, we will never actually wait
      // this long for new signal. We expect to get a signal in at most 1/freq
      StatusCode status =
          BaseStatusSignal.waitForAll(
              2.0 / RobotInfo.DriveInfo.STATUS_SIGNAL_FREQUENCY, m_allStatusSignals);

      doCycle();

      synchronized (this) {
        if (status.isOK()) {
          m_successfulCycles++;
        } else {
          m_failedCycles++;
        }

        double runFinishedAt = Timer.getFPGATimestamp();
        m_loopTimeTracker.sample(runFinishedAt - runStartedAt);
        runStartedAt = runFinishedAt;
      }
    }
  }

  private void doCycle() {
    Rotation2d yawDegrees = m_pigeon.getYaw();
    synchronized (this) {
      m_lastPoseMeters = getPoseMeters();
      m_swerveOdometry.update(yawDegrees, getPositions());
    }
    m_driveController.syncSensors();
    m_driveController.update();

    log();
  }

  private synchronized void log() {
    SmartDashboard.putNumberArray(
        "swerve/odometry",
        ImmutableList.of(
                getPoseMeters().getTranslation().getX(),
                getPoseMeters().getTranslation().getY(),
                getPoseMeters().getRotation().getDegrees())
            .toArray(Double[]::new));
  }

  private SwerveModulePosition[] getPositions() {
    var positions = new SwerveModulePosition[4];
    for (var mod : m_swerveModules) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public synchronized double getUpdateFrequencyMean() {
    return m_loopTimeTracker.getMean();
  }

  public synchronized double getUpdateFrequencyStdDev() {
    return m_loopTimeTracker.getStdDev();
  }

  public synchronized int getSuccessfulCycles() {
    return m_successfulCycles;
  }

  public synchronized int getFailedCycles() {
    return m_failedCycles;
  }
}

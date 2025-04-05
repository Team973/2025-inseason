package com.team973.frc2025.auto.modes;

import com.team973.frc2025.auto.commands.DriveTrajectoryCommand;
import com.team973.frc2025.auto.commands.util.DelayCommand;
import com.team973.frc2025.auto.commands.util.LambdaCommand;
import com.team973.frc2025.shared.RobotInfo.Colors;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.DriveController.ControllerOption;
import com.team973.frc2025.subsystems.SolidSignaler;
import com.team973.frc2025.subsystems.Superstructure;
import com.team973.lib.util.AutoMode;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ScoreTestAuto extends AutoMode {
  public static final SolidSignaler m_finishedDrivingSignaler =
      new SolidSignaler(Colors.GREEN, 500, 8);

  public ScoreTestAuto(Logger logger, DriveController drive, Superstructure superstructure) {
    super(
        logger,
        new Pose2d(17.13, 3.95, Rotation2d.fromDegrees(0)),
        // new ScoreCoralCommand(drive, superstructure, ReefFace.A, ReefLevel.L_4, ReefSide.Left),
        new LambdaCommand(
            () -> {
              drive.setControllerOption(ControllerOption.DriveWithJoysticks);
              drive.getDriveWithJoysticks().updateInput(0.1, 0, 0);
            }),
        new DelayCommand(1.0),
        new LambdaCommand(() -> m_finishedDrivingSignaler.enable()),
        new DriveTrajectoryCommand(drive, "Score-Test", logger.subLogger("Score-Test")));
  }

  public String getName() {
    return "Score Test Auto";
  }
}

package com.team973.frc2025.auto.modes;

import com.team973.frc2025.auto.commands.util.DelayCommand;
import com.team973.frc2025.subsystems.Claw;
import com.team973.lib.util.AutoMode;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;

public class ClawTestAuto extends AutoMode {

  public ClawTestAuto(Logger logger, Claw claw) {
    super(
        logger,
        new Pose2d(),
        // new ClawCommand(claw, ControlStatus.IntakeAndHold),
        // new WaitUntillCoralSeeStateCommand(claw, true),
        // new DelayCommand(1),
        // new ClawCommand(claw, ControlStatus.Score),
        // new WaitUntilCoralScoredComand(claw),
        // new ClawCommand(claw, ControlStatus.IntakeAndHold),
        // new WaitUntillCoralSeeStateCommand(claw, true),
        // new DelayCommand(1),
        // new ClawCommand(claw, ControlStatus.Score),
        // new WaitUntilCoralScoredComand(claw),
        // new ClawCommand(claw, ControlStatus.IntakeAndHold),
        // new WaitUntillCoralSeeStateCommand(claw, true),
        // new DelayCommand(1),
        // new ClawCommand(claw, ControlStatus.Score),
        // new WaitUntilCoralScoredComand(claw),
        // new ClawCommand(claw, ControlStatus.IntakeAndHold),
        // new WaitUntillCoralSeeStateCommand(claw, true),
        // new DelayCommand(1),
        // new ClawCommand(claw, ControlStatus.Score),
        // new WaitUntilCoralScoredComand(claw),
        new DelayCommand(1));
  }

  public String getName() {
    return "Claw Test Auto";
  }
}

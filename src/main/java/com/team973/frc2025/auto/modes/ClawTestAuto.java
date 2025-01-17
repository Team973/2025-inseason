package com.team973.frc2025.auto.modes;

import com.team973.frc2025.auto.commands.ClawCommand;
import com.team973.frc2025.auto.commands.WaitUntillCoralSeeStateCommand;
import com.team973.frc2025.auto.commands.util.DelayCommand;
import com.team973.frc2025.subsystems.Claw;
import com.team973.frc2025.subsystems.Claw.ControlStatus;
import com.team973.lib.util.AutoMode;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;

public class ClawTestAuto extends AutoMode {

  public ClawTestAuto(Logger logger, Claw claw) {
    super(
        logger,
        new Pose2d(),
        new ClawCommand(claw, ControlStatus.Shoot),
        new WaitUntillCoralSeeStateCommand(claw, true),
        new ClawCommand(claw, ControlStatus.Retract),
        new WaitUntillCoralSeeStateCommand(claw, false),
        new ClawCommand(claw, ControlStatus.Shoot),
        new WaitUntillCoralSeeStateCommand(claw, true),
        new ClawCommand(claw, ControlStatus.Retract),
        new WaitUntillCoralSeeStateCommand(claw, false),
        new ClawCommand(claw, ControlStatus.Shoot),
        new WaitUntillCoralSeeStateCommand(claw, true),
        new ClawCommand(claw, ControlStatus.Retract),
        new WaitUntillCoralSeeStateCommand(claw, false),
        new ClawCommand(claw, ControlStatus.Shoot),
        new WaitUntillCoralSeeStateCommand(claw, true),
        new ClawCommand(claw, ControlStatus.Retract),
        new DelayCommand(1));
  }

  public String getName() {
    return "Claw Test Auto";
  }
}

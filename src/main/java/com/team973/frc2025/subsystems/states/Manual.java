package com.team973.frc2025.subsystems.states;

import com.team973.frc2025.subsystems.Superstructure;
import com.team973.lib.util.State;

public class Manual extends State {
  public Manual(Superstructure superstructure) {
    super(superstructure);
  }

  public void onEntrance() {
    getSuperstructure().setManualScore(false);
    getSuperstructure().setManualIntake(false);
  }

  public void run() {
    if (getSuperstructure().getManualScore()) {
      getSuperstructure().clawScore();
    } else if (getSuperstructure().getManualIntake()) {
      getSuperstructure().clawIntake();
    } else {
      getSuperstructure().clawReverse();
    }

    if (getSuperstructure().getManualArmivator()) {
      getSuperstructure().armTargetReefLevel();
      getSuperstructure().elevatorTargetReefLevel();
      getSuperstructure().wristTargetReefLevel();
    } else {
      getSuperstructure().armStow();
      getSuperstructure().elevatorStow();
      getSuperstructure().wristStow();
    }
  }

  public void onExit() {}
}

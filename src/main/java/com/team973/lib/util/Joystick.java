package com.team973.lib.util;

import edu.wpi.first.wpilibj.XboxController;

public class Joystick extends XboxController {
  private Type m_type;

  private boolean m_leftTriggerLast = false;
  private boolean m_rightTriggerLast = false;

  private int m_lastPOV = -1;

  private final Logger m_logger;

  public enum Type {
    XboxController,
    SickStick
  }

  public Joystick(int port, Type joystickType, Logger logger) {
    super(port);
    m_type = joystickType;
    m_logger = logger;
  }

  /**
   * @return The value of raw button 7 if the controller is a sick stick. Whether or not the left
   *     trigger axis is greater than 0.3 if it's an Xbox controller.
   */
  public boolean getLeftTrigger() {
    if (m_type == Type.SickStick) {
      return super.getRawButton(7);
    } else {
      return super.getLeftTriggerAxis() > 0.3;
    }
  }

  /**
   * @return True if the left trigger was pressed last cycle but not this cycle.
   */
  public boolean getLeftTriggerReleased() {
    return (!this.getLeftTrigger() && m_leftTriggerLast);
  }

  /**
   * @return True if the left trigger was pressed this cycle but not last cycle.
   */
  public boolean getLeftTriggerPressed() {
    return (this.getLeftTrigger() && !m_leftTriggerLast);
  }

  /**
   * @return The value of raw button 8 if the controller is a sick stick. Whether or not the right
   *     trigger axis is greater than 0.3 if it's an Xbox controller.
   */
  public boolean getRightTrigger() {
    if (m_type == Type.SickStick) {
      return super.getRawButton(8);
    } else {
      return super.getRightTriggerAxis() > 0.3;
    }
  }

  /**
   * @return True if the right trigger was pressed last cycle but not this cycle.
   */
  public boolean getRightTriggerReleased() {
    return (!this.getRightTrigger() && m_rightTriggerLast);
  }

  /**
   * @return True if the right trigger was pressed this cycle but not last cycle.
   */
  public boolean getRightTriggerPressed() {
    return (this.getRightTrigger() && !m_rightTriggerLast);
  }

  /**
   * @return The value of raw button 5 if the controller is a sick stick. The value of the left
   *     bumper if it's an Xbox controller.
   */
  @Override
  public boolean getLeftBumperButton() {
    if (m_type == Type.SickStick) {
      return super.getRawButton(5);
    } else {
      return super.getLeftBumperButton();
    }
  }

  /**
   * @return The value of raw button 5 pressed if the controller is a sick stick. The value of the
   *     left bumper pressed if it's an Xbox controller.
   */
  @Override
  public boolean getLeftBumperButtonPressed() {
    if (m_type == Type.SickStick) {
      return super.getRawButtonPressed(5);
    } else {
      return super.getLeftBumperButtonPressed();
    }
  }

  /**
   * @return The value of raw button 5 released if the controller is a sick stick. The value of the
   *     left bumper released if it's an Xbox controller.
   */
  @Override
  public boolean getLeftBumperButtonReleased() {
    if (m_type == Type.SickStick) {
      return super.getRawButtonReleased(5);
    } else {
      return super.getLeftBumperButtonReleased();
    }
  }

  /**
   * @return The value of raw button 6 if the controller is a sick stick. The value of the right
   *     bumper if it's an Xbox controller.
   */
  @Override
  public boolean getRightBumperButton() {
    if (m_type == Type.SickStick) {
      return super.getRawButton(6);
    } else {
      return super.getRightBumperButton();
    }
  }

  /**
   * @return The value of raw button 6 pressed if the controller is a sick stick. The value of the
   *     right bumper pressed if it's an Xbox controller.
   */
  @Override
  public boolean getRightBumperButtonPressed() {
    if (m_type == Type.SickStick) {
      return super.getRawButtonPressed(6);
    } else {
      return super.getRightBumperButtonPressed();
    }
  }

  /**
   * @return The value of raw button 6 released if the controller is a sick stick. The value of the
   *     left bumper released if it's an Xbox controller.
   */
  @Override
  public boolean getRightBumperButtonReleased() {
    if (m_type == Type.SickStick) {
      return super.getRawButtonReleased(6);
    } else {
      return super.getRightBumperButtonReleased();
    }
  }

  /**
   * @return The value of raw axis 1.
   */
  public double getLeftXAxis() {
    if (m_type == Type.SickStick) {
      return super.getRawAxis(1);
    } else {
      return super.getRawAxis(0);
    }
  }

  /**
   * @return The value of raw axis 0.
   */
  public double getLeftYAxis() {
    if (m_type == Type.SickStick) {
      return super.getRawAxis(0);
    } else {
      return -super.getRawAxis(1);
    }
  }

  /**
   * @return The value of raw axis 2 if the controller is a sick stick, else the value of raw axis
   *     4.
   */
  public double getRightXAxis() {
    if (m_type == Type.SickStick) {
      return super.getRawAxis(2);
    } else {
      return super.getRawAxis(4);
    }
  }

  public double getRightYAxis() {
    if (m_type == Type.SickStick) {
      return super.getRawAxis(3);
    } else {
      return super.getRawAxis(5);
    }
  }

  /**
   * @return If the POV is equal to 0.
   */
  public boolean getPOVTop() {
    return super.getPOV() == 0;
  }

  /**
   * @return If the POV is equal to 0 and the last POV was equal to -1.
   */
  public boolean getPOVTopPressed() {
    return (m_lastPOV == -1 && getPOVTop());
  }

  /**
   * @return If the POV is equal to 90.
   */
  public boolean getPOVRight() {
    return super.getPOV() == 90;
  }

  /**
   * @return If the POV is equal to 0 and the last POV was equal to -1.
   */
  public boolean getPOVRightPressed() {
    return (m_lastPOV == -1 && getPOVRight());
  }

  /**
   * @return If the POV is equal to 180.
   */
  public boolean getPOVBottom() {
    return super.getPOV() == 180;
  }

  /**
   * @return If the POV is equal to 0 and the last POV was equal to -1.
   */
  public boolean getPOVBottomPressed() {
    return (m_lastPOV == -1 && getPOVBottom());
  }

  /**
   * @return If the POV is equal to 270.
   */
  public boolean getPOVLeft() {
    return super.getPOV() == 270;
  }

  /**
   * @return If the POV is equal to 0 and the last POV was equal to -1.
   */
  public boolean getPOVLeftPressed() {
    return (m_lastPOV == -1 && getPOVLeft());
  }

  public void log() {
    m_logger.log("Left Trigger", this.getLeftTrigger());
    m_logger.log("Left Trigger Released", this.getLeftTriggerReleased());

    m_logger.log("Right Trigger", this.getRightTrigger());
    m_logger.log("Right Trigger Released", this.getRightTriggerReleased());

    m_logger.log("Left Bumper", this.getLeftBumperButton());
    m_logger.log("Left Bumper Pressed", this.getLeftBumperButtonPressed());
    m_logger.log("Left Bumper Released", this.getLeftBumperButtonReleased());

    m_logger.log("Right Bumper", this.getRightBumperButton());
    m_logger.log("Right Bumper Pressed", this.getRightBumperButtonPressed());
    m_logger.log("Right Bumper Released", this.getRightBumperButtonReleased());

    m_logger.log("A Button", this.getAButton());
    m_logger.log("A Button Pressed", this.getAButtonPressed());
    m_logger.log("A Button Released", this.getAButtonReleased());

    m_logger.log("B Button", this.getBButton());
    m_logger.log("B Button Pressed", this.getBButtonPressed());
    m_logger.log("B Button Released", this.getBButtonReleased());

    m_logger.log("X Button", this.getXButton());
    m_logger.log("X Button Pressed", this.getXButtonPressed());
    m_logger.log("X Button Released", this.getXButtonReleased());

    m_logger.log("Y Button", this.getYButton());
    m_logger.log("Y Button Pressed", this.getYButtonPressed());
    m_logger.log("Y Button Released", this.getYButtonReleased());

    m_logger.log("Start Button", this.getStartButton());
    m_logger.log("Start Button Pressed", this.getStartButtonPressed());
    m_logger.log("Start Button Released", this.getStartButtonReleased());

    m_logger.log("Back Button", this.getBackButton());
    m_logger.log("Back Button Pressed", this.getBackButtonPressed());
    m_logger.log("Back Button Released", this.getBackButtonReleased());

    m_logger.log("POV Top", this.getPOVTop());
    m_logger.log("POV Top Pressed", this.getPOVTopPressed());

    m_logger.log("POV Left", this.getPOVLeft());
    m_logger.log("POV Left Pressed", this.getPOVLeftPressed());

    m_logger.log("POV Right", this.getPOVRight());
    m_logger.log("POV Right Pressed", this.getPOVRightPressed());

    m_logger.log("POV Down", this.getPOVBottom());
    m_logger.log("POV Down Pressed", this.getPOVBottomPressed());

    m_logger.log("Right X", getRightXAxis());
    m_logger.log("Right Y", getRightY());

    m_logger.log("Left X", getLeftXAxis());
    m_logger.log("Left Y", getLeftYAxis());
  }

  public void update() {
    m_rightTriggerLast = this.getRightTrigger();
    m_leftTriggerLast = this.getLeftTrigger();

    this.log();

    m_lastPOV = super.getPOV();
  }
}

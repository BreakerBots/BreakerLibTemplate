// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/** Add your docs here. */
public class BreakerXboxController {
   private static final int kButtonA = 1;
   private static final int kButtonB = 2;
   private static final int kButtonX = 3;
   private static final int kButtonY = 4;
   private static final int kBackButton = 7;
   private static final int kStartButton = 8;
   private static final int kRightJoystickButton = 9;
   private static final int kLeftJoystickButton = 10;
   private static final int kDpadUp = 0;
   private static final int kDpadRight = 90;
   private static final int kDpadDown = 180;
   private static final int kDpadLeft = 270;
   
    private XboxController xbox;

    private JoystickButton buttonA;
    private JoystickButton buttonB;
    private JoystickButton buttonX;
    private JoystickButton buttonY;
    private JoystickButton backButton;
    private JoystickButton startButton;
    private JoystickButton leftJoystickButton;
    private JoystickButton rightJoystickButton;
    private POVButton dUp;
    private POVButton dRight;
    private POVButton dDown;
    private POVButton dLeft;
    public BreakerXboxController(int controllerPort) {
        xbox = new XboxController(controllerPort);
        buttonA = new JoystickButton(xbox, kButtonA);
        buttonB = new JoystickButton(xbox, kButtonB);
        buttonX = new JoystickButton(xbox, kButtonX);
        buttonY = new JoystickButton(xbox, kButtonY);
        backButton = new JoystickButton(xbox, kBackButton);
        startButton = new JoystickButton(xbox, kStartButton);
        leftJoystickButton = new JoystickButton(xbox, kLeftJoystickButton);
        rightJoystickButton = new JoystickButton(xbox, kRightJoystickButton);
        dUp = new POVButton(xbox, kDpadUp);
        dRight = new POVButton(xbox, kDpadRight);
        dDown = new POVButton(xbox, kDpadDown);
        dLeft = new POVButton(xbox, kDpadLeft);
        
    }

    public void whenPressedButtonA(Command command, boolean isInteruptable) {
        buttonA.whenPressed(command, isInteruptable);
    }

    public void whenPressedButtonB(Command command, boolean isInteruptable) {
        buttonB.whenPressed(command, isInteruptable);
    }

    public void whenPressedButtonX(Command command, boolean isInteruptable) {
        buttonX.whenPressed(command, isInteruptable);
    }

    public void whenPressedButtonY(Command command, boolean isInteruptable) {
        buttonY.whenPressed(command, isInteruptable);
    }

    public void whenPressedBackButton(Command command, boolean isInteruptable) {
        backButton.whenPressed(command, isInteruptable);
    }

    public void whenPressedStartButton(Command command, boolean isInteruptable) {
        startButton.whenPressed(command, isInteruptable);
    }

    public void whenHeldButtonA(Command command, boolean isInteruptable) {
        buttonA.whenHeld(command, isInteruptable);
    }

    public void whenHeldButtonB(Command command, boolean isInteruptable) {
        buttonB.whenHeld(command, isInteruptable);
    }

    public void whenHeldButtonX(Command command, boolean isInteruptable) {
        buttonX.whenHeld(command, isInteruptable);
    }

    public void whenHeldButtonY(Command command, boolean isInteruptable) {
        buttonY.whenHeld(command, isInteruptable);
    }

    public void whenHeldBackButton(Command command, boolean isInteruptable) {
        backButton.whenHeld(command, isInteruptable);
    }

    public void whenHeldStartButton(Command command, boolean isInteruptable) {
        startButton.whenHeld(command, isInteruptable);
    }

    public void whenReleasedButtonA(Command command, boolean isInteruptable) {
        buttonA.whenReleased(command, isInteruptable);
    }

    public void whenReleasedButtonB(Command command, boolean isInteruptable) {
        buttonB.whenReleased(command, isInteruptable);
    }

    public void whenReleasedButtonX(Command command, boolean isInteruptable) {
        buttonX.whenReleased(command, isInteruptable);
    }

    public void whenReleasedButtonY(Command command, boolean isInteruptable) {
        buttonY.whenReleased(command, isInteruptable);
    }

    public void whenReleasedBackButton(Command command, boolean isInteruptable) {
        backButton.whenReleased(command, isInteruptable);
    }

    public void whenReleasedStartButton(Command command, boolean isInteruptable) {
        startButton.whenReleased(command, isInteruptable);
    }

    
}

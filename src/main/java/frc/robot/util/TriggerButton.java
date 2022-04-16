package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerButton extends Trigger {
    private GenericHID gamepad;
    private boolean left = false;
    
    public TriggerButton(GenericHID gamepad, boolean left) {
        this.gamepad = gamepad;
        this.left = left;
    }

    @Override
    public boolean get() {
        if (left) {
            return gamepad.getRawAxis(2) > 0.6;
        }
        return gamepad.getRawAxis(3) > 0.6;
    }
    
}

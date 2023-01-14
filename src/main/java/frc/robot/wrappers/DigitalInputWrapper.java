package frc.robot.wrappers;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalInputWrapper {
    DigitalInput digitalInput;

    public DigitalInputWrapper(int ID) {
        digitalInput = new DigitalInput(ID);
    }

    public boolean get() {
        return digitalInput.get();
    }
}

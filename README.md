# Prototype_Chassis

# v1.0 LockPID Code Finish & Some Code Optimization

Commiter : **Moyu**

## Features

1. Finish `LockPID.java` Code
    <span style="color:orange">HINT： Constants didn't finish</span>, only make sure it can automatically correct its position

2. Add button to set encoder to zero
    ```java
        new JoystickButton(driverJoystick, OIConstants.Btn_B)
            .onTrue( new RunCommand( () -> { m_drive.resetEncoders();}, m_drive));
    ```

## Changes

* `m_drive.setDefultCommand` From TankDrive change to ArcadeDrive
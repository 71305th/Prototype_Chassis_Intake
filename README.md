# Prototype_Chassis_Intake

# v1.2 Add Some Details

Commiter : **Moyu**

## Freatures

1. 
```java
new JoystickButton(driverJoystick, OIConstants.Btn_A)
    .onTrue(m_setPoint);

new JoystickButton(driverJoystick, OIConstants.Btn_B)
    .onTrue( new RunCommand( () -> { m_drive.resetEncoders(); }, m_drive));
```

2. Add ResetEncoder() in `LockID -> Intake`
```java
drive.resetEncoders();
```

# v1.1 Adjust button bindings & Clean unused codes

Commiter: **Hong**

## Features

1. Adjust button bindings
    * A: Toggle LockPID function.
    * B: Toggle front intake open or close.
    * X: Toggle Rear intake open or close.
    * RT: Toggle front intake down or up.
    * LT: Toggle rear intake down or up.
    * RB: When pressed, boost the drivetrain (speed up from 0.7 to 0.8, rot 0.85 to 0.95.)

2. Clean unused codes: Remove unused imports, functions, constants and something like that.

## Changes

1. Corrected motor reverses
2. Finish LockPID function but may still need some adjustment
3. Finish Ramsete PathFollowing Cmd, waiting for test
4. Finish intake code, waiting for test

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

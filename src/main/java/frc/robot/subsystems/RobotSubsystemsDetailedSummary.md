# Robot Subsystems – Detailed Summary

## Table of Main Subsystems

| Subsystem       | Main Purpose & Responsibilities | Key Properties           | Key Methods/Commands                |
|-----------------|--------------------------------|-------------------------|-------------------------------------|
| **Intake**      | Collect coral from ground; controls rollers, pivot, sensors | `rollers`, `pivot`, `indexer`, `sensors` | `intake()`, `intakeVertical()`, `drop()`, `stow()`, `transfer()`, `getCoralLocation()` |
| **Elevator**    | Elevate coral to scoring heights; ensure closed-loop position; limit switches for safety | `leftMotor`, `rightMotor`, `bottomLimitSwitch`, `topLimitSwitch`, `targetPosition` | `coralL1()`, `coralL2()`, `coralL3()`, `coralL4()`, `algaeIntake()`, `stow()`, etc. |
| **Manipulator** | Secure and place coral; manipulates coral/algae via grabber and pivot | `pivot`, `grabber` | `placeCoralL1()`, `placeCoralL23()`, `placeCoralL4()`, `intakeCoral()`, `pickupGroundAlgae()`, `pickupReefAlgae()`, `dropReefAlgae()`, `stow()` |
| **LEDs**        | Show robot state, feedback, and visuals for drivers | `strip`, `buffer`, `state`, color/pattern constants | `setStateCommand(state)`, `setState(state)`, custom pattern/gradient creation         |
| **Controls**    | Handle driver/operator input, map buttons, setup command bindings | `operator`, `driver`, `xBoxSwerve` | `configureBindings(...)`, `getSwerveController()`                                     |

---

## Interactions Diagram

```mermaid
flowchart LR
    I[Intake] --> E[Elevator]
    E --> M[Manipulator]
    C[Controls] -.-> I
    C -.-> E
    C -.-> M
    C -.-> L[LEDs]
    L <-->|State Feedback| C
    M -->|Score/Release| X[Field]
    I -.->|Sensors| C
```

---

## Subsystem Class Diagrams

### Intake Subsystem (src/main/java/frc/robot/subsystems/intake/Intake.java)
```mermaid
classDiagram
    class Intake {
      +IntakeRollers rollers
      +IntakePivot pivot
      +Indexer indexer
      +IntakeSensors sensors
      +Command intake()
      +Command intakeVertical()
      +Command drop()
      +Command stow()
      +Command transfer()
      +CoralLocation getCoralLocation()
    }
    Intake --> IntakeRollers
    Intake --> IntakePivot
    Intake --> Indexer
    Intake --> IntakeSensors
```

### Elevator Subsystem (src/main/java/frc/robot/subsystems/elevator/Elevator.java)
```mermaid
classDiagram
    class Elevator {
      +TalonFX leftMotor
      +TalonFX rightMotor
      +DigitalInput bottomLimitSwitch
      +DigitalInput topLimitSwitch
      +Distance targetPosition
      +boolean elevatorZeroed
      +Grabber grabber
      +Command coralL1()
      +Command coralL2()
      +Command coralL3()
      +Command coralL4()
      +Command algaeIntake()
      +Command stow()
      ...
    }
```

### Manipulator Subsystem (src/main/java/frc/robot/subsystems/manipulator/Manipulator.java)
```mermaid
classDiagram
    class Manipulator {
      +ManipulatorPivot pivot
      +Grabber grabber
      +Command placeCoralL1()
      +Command placeCoralL23()
      +Command placeCoralL4()
      +Command intakeCoral()
      +Command pickupGroundAlgae()
      +Command pickupReefAlgae()
      +Command dropReefAlgae()
      +Command stow()
    }
    Manipulator --> ManipulatorPivot
    Manipulator --> Grabber
```

### LEDs Subsystem (src/main/java/frc/robot/subsystems/leds/LEDs.java)
```mermaid
classDiagram
    class LEDs {
      -AddressableLED strip
      -AddressableLEDBuffer buffer
      -State state
      -LEDPattern m_rainbow
      -LEDPattern m_scrollingRainbow
      +static Command setStateCommand(State)
      +static void setState(State)
      -Color convertVisibleColorToDriverColor(Color)
      ...
    }
```

### Controls Subsystem (src/main/java/frc/robot/subsystems/Controls.java)
```mermaid
classDiagram
    class Controls {
      +CommandXboxController operator
      +CommandXboxController driver
      -XBoxSwerve xBoxSwerve
      +XBoxSwerve getSwerveController()
      +void configureBindings(...)
    }
```

---

## Notes

- The class diagrams above show relationships and key commands/properties; see the [full source files](https://github.com/snehalmthakkar/Code-2025/tree/main/src/main/java/frc/robot/subsystems) for all details.
- *Intake*, *Elevator*, and *Manipulator* subsystems use command-based architecture (WPILib) for sophisticated sequencing and safety.
- *LEDs* subsystem provides feedback synchronized with robot state (such as aligning, scoring, or error states).
- *Controls* subsystem is the gateway for all manual and autonomous control logic, safely invoking commands across all mechanical subsystems.

_These details are based on code search results. For full member and method listings, see individual Java files in your [subsystems directory](https://github.com/snehalmthakkar/Code-2025/tree/main/src/main/java/frc/robot/subsystems)._

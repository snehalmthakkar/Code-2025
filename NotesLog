First step I forked the team 6962 code 2025 repo so I can mess with it.

I started asking the copilot within github to start adding comments to the files. It was a little annoying because I had to do it one folder at a time (the free version that I have will only do ten files at a time, student version should do 50 files).- I had to copy paste the code into the files (this is a me problem, since I don't have vs code I was doing this in online github interface. If you are doing this from copilot integrated with vs code it will do the copying, you will just have to commit the changes).- I managed to do it for all commands and auto files. I will probably go through and do other files over the weekend.- I also used GitDiagram to generate the class diagram for all the files. Again not perfect, but it is a starting version. 

I also started asking copilot to explain to me how some of the constants are used across the code. Sadly the ten files limit makes this a little useless, but I think by asking it more specific questions I might be able to make it give me more useful answers.

Here is an example question and answer: can you tell me where these constants are referred to from other part of the codebase? We can start with elevator constants

Here are several places in your codebase where the `Constants.ELEVATOR` constants are used:

---

### 1. **Subsystem Implementation**
- **Elevator.java** ([link](https://github.com/snehalmthakkar/Code-2025/blob/b5a1f6373e6510f49b9895ae3506bf3df5f914af/src/main/java/frc/robot/subsystems/elevator/Elevator.java#L256-L360))
  - Sets elevator positions, e.g. `startPositionControl(ElevatorConstants.MIN_HEIGHT, true)`
  - Command methods like `coralL1()`, `algaeL2()`, and `ready()` use heights from `Constants.ELEVATOR` and its nested classes.
  - Example: `moveToPosition(Constants.ELEVATOR.CORAL.L1_HEIGHT)`

---

### 2. **Command Classes**
- **SafeSubsystems.java** ([link](https://github.com/snehalmthakkar/Code-2025/blob/b5a1f6373e6510f49b9895ae3506bf3df5f914af/src/main/java/frc/robot/commands/SafeSubsystems.java#L1-L85))
  - Used to calculate safe angles for the manipulator based on the elevator height.
  - Example: `ELEVATOR.CORAL.INTAKE_HEIGHT.plus(Inches.of(4))`

- **PieceCombos.java** ([link](https://github.com/snehalmthakkar/Code-2025/blob/b5a1f6373e6510f49b9895ae3506bf3df5f914af/src/main/java/frc/robot/commands/PieceCombos.java#L1-L97))
  - Used to coordinate elevator and manipulator actions for scoring/intaking.
  - Example: `ELEVATOR.CORAL.L1_HEIGHT`, `ELEVATOR.CORAL.INTAKE_HEIGHT`

- **IntakeCommands.java** ([link](https://github.com/snehalmthakkar/Code-2025/blob/b5a1f6373e6510f49b9895ae3506bf3df5f914af/src/main/java/frc/robot/commands/IntakeCommands.java#L1-L68))
  - Used in intake and transfer command sequencing.
  - Example: `ELEVATOR.CORAL.INTAKE_HEIGHT`

---

### 3. **Simulation**
- **SimElevator.java** ([link](https://github.com/snehalmthakkar/Code-2025/blob/b5a1f6373e6510f49b9895ae3506bf3df5f914af/src/main/java/frc/robot/subsystems/elevator/SimElevator.java#L1-L75))
  - Elevator simulation uses MIN_HEIGHT and MAX_HEIGHT for bounds and simulation logic.
  - Example: `ElevatorConstants.MIN_HEIGHT.in(Meters)`, `ElevatorConstants.MAX_HEIGHT.in(Meters)`

---

### 4. **Autonomous Logic**
- **Autonomous.java** ([link](https://github.com/snehalmthakkar/Code-2025/blob/b5a1f6373e6510f49b9895ae3506bf3df5f914af/src/main/java/frc/robot/auto/Autonomous.java#L1-L99))
  - Uses `ELEVATOR` for autonomous routines and logic (e.g., scoring/intake heights).

---

### 5. **Controls**
- **Controls.java** ([link](https://github.com/snehalmthakkar/Code-2025/blob/b5a1f6373e6510f49b9895ae3506bf3df5f914af/src/main/java/frc/robot/subsystems/Controls.java#L1-L96))
  - Binds operator controls to elevator actions and heights using constants.

---

**Note:**  
Only the first 10 results are shown here. There may be additional usages throughout the codebase.  
To see more, [search for `Constants.ELEVATOR` in the repository on GitHub](https://github.com/snehalmthakkar/Code-2025/search?q=Constants.ELEVATOR).



Let me know if you'd like a more detailed explanation of how these constants are used in any specific file!

/**
 * AutoChooser.java
 *
 * This class provides a mechanism to select and manage autonomous routines for the robot using the
 * SmartDashboard SendableChooser. It allows for dynamic selection of different autonomous routines at runtime,
 * logs the current selection, and provides a fallback mechanism if the selected or default routine does not exist.
 * It also prepares the selected autonomous routine for execution and provides the appropriate Command
 * to be scheduled in autonomous mode.
 *
 * Inner class {@link Auto} represents an autonomous routine with the ability to prepare or disallow preparation.
 */

package frc.robot.auto;

import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The AutoChooser subsystem allows users to select between different autonomous routines
 * via the SmartDashboard. It also manages the preparation and retrieval of the selected
 * autonomous Command.
 */
public class AutoChooser extends SubsystemBase {
    private Map<String, Auto> autos;
    private SendableChooser<String> sendableChooser;
    private String defaultAuto;

    private static Auto BACKUP_AUTO = new Auto("Backup Auto", Commands.print("No autonomous available. Selected and default autos do not exist."));

    private Auto preparedAuto;

    /**
     * Constructs an AutoChooser.
     * Initializes the chooser with a list of autonomous routines and a default selection.
     * Adds all routines to the SmartDashboard chooser and logs selection info.
     *
     * @param autosList   List of available autonomous routines (Auto objects)
     * @param defaultAuto The name of the default autonomous routine
     */
    public AutoChooser(List<Auto> autosList, String defaultAuto) {
        this.autos = autosList.stream().collect(Collectors.toMap(Auto::getName, auto -> auto));
        this.defaultAuto = defaultAuto;

        sendableChooser = new SendableChooser<>();
        sendableChooser.setDefaultOption(defaultAuto, defaultAuto);

        for (String autoName : autos.keySet()) {
            sendableChooser.addOption(autoName, autoName);
        }

        SmartDashboard.putData("Autonomous", sendableChooser);

        Logger.logString("AutoChooser/selectedAuto", () -> sendableChooser.getSelected());
        Logger.logString("AutoChooser/defaultAuto", () -> defaultAuto);
        Logger.logBoolean("AutoChooser/selectedAutoExists", () -> autos.containsKey(sendableChooser.getSelected()));
        Logger.logBoolean("AutoChooser/defaultAutoExists", () -> autos.containsKey(defaultAuto));

        prepareAuto();
    }

    /**
     * Called periodically by the scheduler.
     * Ensures the selected autonomous routine is prepared if changed.
     */
    @Override
    public void periodic() {
        prepareAuto();
    }

    /**
     * Prepares the currently selected autonomous routine for execution.
     * If the selected auto has changed, re-initializes the preparedAuto.
     * Falls back to the default or backup auto if necessary.
     */
    private void prepareAuto() {
        if (preparedAuto != null && preparedAuto.getName().equals(sendableChooser.getSelected())) return;

        String autoName = sendableChooser.getSelected();
        Auto auto = autos.getOrDefault(autoName, autos.getOrDefault(defaultAuto, BACKUP_AUTO));

        preparedAuto = auto.prepare();
    }

    /**
     * Returns the Command for the currently prepared autonomous routine.
     * Ensures that the current selection is prepared before returning.
     *
     * @return The autonomous Command to be scheduled
     */
    public Command getAutonomousCommand() {
        prepareAuto();
        return preparedAuto.getCommand();
    }

    /**
     * Represents a single autonomous routine, with a name and a supplier for the Command.
     * Supports lazy or eager preparation of the Command instance.
     */
    public static class Auto {
        private String name;
        private Supplier<Command> commandSupplier;
        private boolean allowPrepare = true;

        /**
         * Constructs an Auto with a name and a Command supplier.
         *
         * @param name Name of the autonomous routine
         * @param commandSupplier Supplier that provides a new Command instance
         */
        public Auto(String name, Supplier<Command> commandSupplier) {
            this.name = name;
            this.commandSupplier = commandSupplier;
        }

        /**
         * Constructs an Auto with a name and a Command.
         * The Command is wrapped in a supplier.
         *
         * @param name Name of the autonomous routine
         * @param command Command instance
         */
        public Auto(String name, Command command) {
            this(name, () -> command);
        }

        /**
         * Constructs an Auto using the Command's name as the auto name.
         *
         * @param command Command instance
         */
        public Auto(Command command) {
            this(command.getName(), () -> command);
        }

        /**
         * @return Name of the autonomous routine
         */
        public String getName() {
            return name;
        }

        /**
         * @return The Command instance for this autonomous routine
         */
        public Command getCommand() {
            return commandSupplier.get();
        }

        /**
         * Prevents this Auto from being re-prepared in the future.
         * (Useful if you want only a single instance of the Command.)
         *
         * @return this Auto instance (for chaining)
         */
        public Auto disallowPrepare() {
            allowPrepare = false;
            return this;
        }

        /**
         * Prepares a new Auto instance if allowed; otherwise returns this one.
         * This is useful for cases where you want a fresh Command each time.
         *
         * @return A new Auto instance or this one, depending on allowPrepare
         */
        public Auto prepare() {
            if (allowPrepare) return new Auto(name, commandSupplier.get());
            else return this;
        }
    }
}

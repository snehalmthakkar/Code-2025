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

public class AutoChooser extends SubsystemBase {
    private Map<String, Auto> autos;
    private SendableChooser<String> sendableChooser;
    private String defaultAuto;

    private static Auto BACKUP_AUTO = new Auto("Backup Auto", Commands.print("No autonomous available. Selected and default autos do not exist."));

    private Auto preparedAuto;

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

    @Override
    public void periodic() {
        prepareAuto();
    }

    private void prepareAuto() {
        if (preparedAuto != null && preparedAuto.getName().equals(sendableChooser.getSelected())) return;

        String autoName = sendableChooser.getSelected();
        Auto auto = autos.getOrDefault(autoName, autos.getOrDefault(defaultAuto, BACKUP_AUTO));

        preparedAuto = auto.prepare();
    }

    public Command getAutonomousCommand() {
        prepareAuto();
        
        return preparedAuto.getCommand();
    }

    public static class Auto {
        private String name;
        private Supplier<Command> commandSupplier;
        private boolean allowPrepare = true;

        public Auto(String name, Supplier<Command> commandSupplier) {
            this.name = name;
            this.commandSupplier = commandSupplier;
        }

        public Auto(String name, Command command) {
            this(name, () -> command);
        }

        public Auto(Command command) {
            this(command.getName(), () -> command);
        }

        public String getName() {
            return name;
        }

        public Command getCommand() {
            return commandSupplier.get();
        }

        public Auto disallowPrepare() {
            allowPrepare = false;
            return this;
        }

        public Auto prepare() {
            if (allowPrepare) return new Auto(name, commandSupplier.get());
            else return this;
        }
    }
}

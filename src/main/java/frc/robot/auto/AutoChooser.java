package frc.robot.auto;

import java.util.Map;
import java.util.function.Supplier;

import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoChooser extends SubsystemBase {
    private Map<String, Supplier<Command>> autos;
    private SendableChooser<String> sendableChooser;
    private String defaultAuto;

    private Command preparedAutoCommand;
    private String preparedAutoName;

    public AutoChooser(Map<String, Supplier<Command>> autos, String defaultAuto) {
        this.autos = autos;
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
        if (preparedAutoName != null && preparedAutoName.equals(sendableChooser.getSelected())) return;

        String autoName = sendableChooser.getSelected();

        preparedAutoName = autoName;
        preparedAutoCommand = autos.getOrDefault(autoName, autos.getOrDefault(defaultAuto, () -> Commands.print("No autonomous available. Selected and default autos do not exist."))).get();
    }

    public Command getAutonomousCommand() {
        prepareAuto();
        
        return preparedAutoCommand;
    }
}

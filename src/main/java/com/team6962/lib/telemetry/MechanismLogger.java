package com.team6962.lib.telemetry;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public final class MechanismLogger {
    private static final double ORIGIN_X = 50;
    private static final double ORIGIN_Y = 10;
    private static final double MAX_Y = 100;

    private static final Mechanism2d mechanism2d = new Mechanism2d(ORIGIN_X * 2, MAX_Y);
    private static final Map<String, MechanismLigament2d> ligaments = new HashMap<>();
    private static final Set<String> roots = new HashSet<>();

    public static void start() {
        SmartDashboard.putData("Mechanisms", mechanism2d);
    }

    public static MechanismRoot2d getRoot(String name, double x, double y) {
        MechanismRoot2d root;
        if (roots.contains(name)) {
            root = mechanism2d.getRoot(name, ORIGIN_X, ORIGIN_Y); // x and y are unused
            root.setPosition(x, y);
            return root;
        }

        return mechanism2d.getRoot(name, x + ORIGIN_X, y + ORIGIN_Y);
    }

    public static MechanismRoot2d getRoot(String name) {
        return mechanism2d.getRoot(name, ORIGIN_X, ORIGIN_Y); // if root already exists, x and y are unused. Otherwise,
                                                              // they will be overriden later.
    }

    private static MechanismLigament2d getLigament(String name, Supplier<MechanismLigament2d> generator, Consumer<MechanismLigament2d> modifier) {
        if (!ligaments.containsKey(name)) {
            ligaments.put(name, generator.get());
        }
        
        return ligaments.get(name);
    }

    public static MechanismLigament2d getLigament(String name, double length, double angle, double lineWidth, Color8Bit color) {
        return getLigament(
            name,
            () -> new MechanismLigament2d(name, length, angle, lineWidth, color),
            ligament -> {
                ligament.setLength(length);
                ligament.setAngle(angle);
                ligament.setLineWeight(lineWidth);
                ligament.setColor(color);
            }
        );
    }

    public static MechanismLigament2d getLigament(String name, double length, double angle) {
        return getLigament(
            name,
            () -> new MechanismLigament2d(name, length, angle),
            ligament -> {
                ligament.setLength(length);
                ligament.setAngle(angle);
            }
        );
    }

    public static MechanismLigament2d getLigament(String name) {
        return getLigament(
            name,
            () -> new MechanismLigament2d(name, 0, 0),
            ligament -> {}
        );
    }

    public static void addDynamicLength(MechanismLigament2d ligament, Supplier<Distance> lengthSupplier) {
        Logger.addUpdate("MechanismLogger/" + ligament.getName(), () -> ligament.setLength(lengthSupplier.get().in(Inches)));
    }

    public static void addDynamicAngle(MechanismLigament2d ligament, Supplier<Angle> angleSupplier) {
        Logger.addUpdate("MechanismLogger/" + ligament.getName(), () -> ligament.setAngle(angleSupplier.get().in(Degrees)));
    }

    public static void addDyanmicPosition(MechanismRoot2d root, Supplier<Distance> xSupplier, Supplier<Distance> ySupplier) {
        Logger.addUpdate("MechanismLogger/" + root.getName(), () -> root.setPosition(xSupplier.get().in(Inches) + ORIGIN_X, ySupplier.get().in(Inches) + ORIGIN_Y));
    }

    private MechanismLogger() {}
}

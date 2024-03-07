package frc.utils;

import edu.wpi.first.wpilibj2.command.Command;

public class CustomCommand extends Command {
    private final Runnable onEnd;
    private final Runnable toRun;

    public CustomCommand(Runnable toRun, Runnable onEnd) {
        this.onEnd = onEnd;
        this.toRun = toRun;
    }

    @Override
    public void initialize() {
        toRun.run();
    }

    @Override
    public void end(boolean interrupted) {
        onEnd.run();
    }
}
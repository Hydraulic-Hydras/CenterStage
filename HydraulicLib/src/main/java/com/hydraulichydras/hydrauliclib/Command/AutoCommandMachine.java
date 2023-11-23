package com.hydraulichydras.hydrauliclib.Command;

import java.util.ArrayList;

public class AutoCommandMachine {
    private ArrayList<CommandSequence> commandSequences = new ArrayList<>();
    private int currentCommandIndex;

    public AutoCommandMachine() { this.currentCommandIndex = 0; }

    public AutoCommandMachine addCommandSequence(CommandSequence commandSequence) {
        commandSequences.add(commandSequence);
        return this;
    }

    public AutoCommandMachine build() { return this; }

    public int getCurrentCommandIndex() { return currentCommandIndex; }

    public void reset() { currentCommandIndex = 0; }

    public void run(boolean driveIsBusy) {
        CommandSequence currentCommand = commandSequences.get(currentCommandIndex);

        if (currentCommand.hasCompleted && !driveIsBusy) {
            currentCommand.trigger();
        }

        if (currentCommandIndex == commandSequences.size()-1) {
            currentCommandIndex = 0;
        } else {
            currentCommandIndex++;
        }
    }
}
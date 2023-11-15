package com.hydraulichydras.hydrauliclib.Command;


import com.hydraulichydras.hydrauliclib.Input.GamepadStatic;

public class CommandSequenceTrigger {


    private CommandSequence commandSequence;
    public GamepadStatic.Input triggerCondition;

    public CommandSequenceTrigger(CommandSequence commandSequence) {
        this.commandSequence = commandSequence;
        this.triggerCondition = GamepadStatic.Input.NONE;
    }

    public CommandSequenceTrigger(CommandSequence commandSequence, GamepadStatic.Input triggerCondition) {
        this.commandSequence = commandSequence;
        this.triggerCondition = triggerCondition;
    }

    public void trigger() {
        if (commandSequence.hasCompleted) {
            commandSequence.run();
        }
    }

}

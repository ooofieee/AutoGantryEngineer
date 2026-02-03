#pragma once

enum Phase{
    PHASE_INIT,
    PHASE_ALLIGN,
    PHASE_PUSH,
    PHASE_ROTATE,
    PHASE_SPIN
};

class PhaseInterface {
private:
    Phase current_phase;

public:

    PhaseInterface() : current_phase(PHASE_INIT) {}
    ~PhaseInterface() = default;

    Phase getCurrentPhase() const {
        return current_phase;
    }

    void setCurrentPhase(Phase phase) {
        current_phase = phase;
    }

    void advancePhase() {
        if (current_phase < PHASE_SPIN) {
            current_phase = static_cast<Phase>(static_cast<int>(current_phase) + 1);
        }
    }

    void resetPhase() {
        current_phase = PHASE_INIT;
    }

};

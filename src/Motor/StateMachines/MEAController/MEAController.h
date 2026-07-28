#pragma once

#include <Motor/StateMachines/MEAController/MEAControllerData.h>
#include <algorithms/MEA/MEA.h>
#include <algorithms/MEA/MEAData.h>
#include <events/FSM.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Motor
{

class BoardScheduler;
class FiringSequenceHSM;
class Sensors;

class MEAController
    : public Boardcore::FSM<MEAController>,
      public Boardcore::InjectableWithDeps<Sensors, BoardScheduler,
                                           FiringSequenceHSM>
{
public:
    MEAController();

    virtual ~MEAController() noexcept = default;

    [[nodiscard]] bool start() override;

    MEAControllerState getMEAControllerState();
    Boardcore::MEAState getMEAState();

private:
    void update();
    void calibrate();

    void state_init(const Boardcore::Event& event);
    void state_calibrate(const Boardcore::Event& event);
    void state_ready(const Boardcore::Event& event);
    void state_active(const Boardcore::Event& event);
    void state_end(const Boardcore::Event& event);

    MEA::MEA mea;

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("mea");

    void updateAndLogStatus(MEAControllerState state);
    std::atomic<MEAControllerState> state{MEAControllerState::INIT};

    miosix::FastMutex meaMutex;
};
}  // namespace Motor

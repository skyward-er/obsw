#pragma once

#include <Motor/Sensors/Sensors.h>
#include <Motor/StateMachines/MEAController/MEAControllerData.h>
#include <Motor/BoardScheduler.h>
#include <algorithms/MEA/MEA.h>
#include <events/FSM.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Motor
{
class MEAController : public Boardcore::FSM<MEAController>,
                      public Boardcore::InjectableWithDeps<Sensors, BoardScheduler>
{
public:

    MEAController();

    virtual ~MEAController() noexcept = default;

    [[nodiscard]] bool start() override;

    MEAControllerState getMEAControllerState();

private:
    void update();

    void state_init(const Boardcore::Event& event);
    void state_ready(const Boardcore::Event& event);
    void state_active(const Boardcore::Event& event);
    void state_end(const Boardcore::Event& event);

    //MEA::MEA mea;

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("mea");

    void updateAndLogStatus(MEAControllerState state);
    std::atomic<MEAControllerState> state{MEAControllerState::INIT};

    miosix::FastMutex meaMutex;
    MEAControllerState state;
    
};
}  // namespace Motor


#include <Groundstation/ARPGS/Base/Hub.h>
#include <Groundstation/Common/GsEntryCommon.h>
#include <common/Events.h>
#include <diagnostic/PrintLogger.h>
#include <events/EventBroker.h>
#include <miosix.h>
#include <utils/DependencyManager/DependencyManager.h>

using namespace Boardcore;
using namespace miosix;
using namespace Groundstation;

/**
 * @brief Radio-only GS entrypoint.
 *
 * For ground stations acting purely as a radio, so no ARP hardware attached
 * (such as no motors, no steppers, etc. :)). See arp-gs-entry.cpp for the
 * ARP-capable variant (both build up on the same GsEntryCommon).
 *
 * Initializes the shared software modules (radios, ethernet, serial)
 *  - Green LED is turned on when main radio is detected
 *  - Yellow LED is turned on when payload radio is detected
 *  - Orange LED is turned on when ethernet is detected
 * When done the red LED is fixed. Blinks if a module failed to init.
 */
int main()
{
    ledOff();

    DipStatus dipRead = readDipStatus();
    dipRead.print(std::cout);

    DependencyManager manager;
    PrintLogger logger = Logging::getLogger("radio_gs");

    CommonModules mods{};
    bool ok = insertCommonModules(manager, dipRead, mods);

    HubBase* hub = new GroundstationBase::Hub();
    ok &= manager.insert<HubBase>(hub);

    if (!ok)
    {
        std::cout << "[error] Failed to insert all modules!" << std::endl;
        errorLoop();
    }

    LOG_DEBUG(logger, "[debug] All modules inserted correctly!\n");

    if (!manager.inject())
    {
        std::cout << "[error] Failed to inject the dependencies!" << std::endl;
        errorLoop();
    }

    ledOn();
    Thread::sleep(2000);
    ledOff();

    bool initFatal = false;
    bool started   = startCommonModules(mods, dipRead, logger, initFatal);

    if (!started)
    {
        std::cout << "GS: could not start all modules successfully!"
                  << std::endl;
        errorLoop();
    }

    LOG_INFO(logger, "All modules started successfully!\n");

    reportCommonStatus(mods.boardStatus, logger);

    // If the dip switch says ARP mode is ON, the wrong firmware is flashed on
    // this board.
    if (!checkDipMatchesBinary(dipRead, /*is tihs arp? */ false, logger))
    {
        std::cout << "GS initialization warning: dip switch / firmware "
                     "mismatch (see above). This board will run as a "
                     "radio-only relay regardless of the dip switch setting."
                  << std::endl;
    }

    led3On();  //< fix RED led (CU)
    idleLoop();
    return 0;
}

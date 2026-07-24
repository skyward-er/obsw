#pragma once

#include <Groundstation/ArpGS/BoardStatus.h>
#include <Groundstation/ArpGS/Buses.h>
#include <Groundstation/ArpGS/Ports/Ethernet.h>
#include <Groundstation/ArpGS/Ports/SerialArpGS.h>
#include <Groundstation/ArpGS/Radio/Radio.h>
#include <Groundstation/Automated/BoardScheduler.h>
#include <Groundstation/Common/Ports/EthernetSniffer.h>
#include <diagnostic/PrintLogger.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <iostream>

namespace Groundstation
{

/**
 * @brief Dip switch status for the GS board.
 *
 * Dipswitch configuration
 *  arp mb  pb  mtx ptx ip3 ip2 ip1
 * | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
 * ---------------------------------
 * | I | I | I | I | I | I | I | I |
 * | O | O | O | O | O | O | O | O |
 * ---------------------------------
 * | H | G | F | E | D | C | B | A |
 */
struct DipStatus
{
    bool isARP;
    bool mainHasBackup;
    bool payloadHasBackup;
    bool mainTXenable;
    bool payloadTXenable;
    uint8_t ipConfig;

    void print(std::ostream& os) const
    {
        os << "Dipswitch state:"
           << "\n\tARP mode:             " << isARP
           << "\n\tMain backup radio:    " << mainHasBackup
           << "\n\tPayload backup radio: " << payloadHasBackup
           << "\n\tMain TX enabled:      " << mainTXenable
           << "\n\tPayload TX enabled:   " << payloadTXenable
           << "\n\tIP offset:            " << (int)ipConfig << std::endl;
    }
};

/// Reads the physical dip switch and decodes it into a DipStatus.
DipStatus readDipStatus();

/**
 * @brief Modules shared by every GS board (both 'it's just a radio' and ARP)
 */
struct CommonModules
{
    BoardScheduler* scheduler         = nullptr;
    Buses* buses                      = nullptr;
    ArpGS::SerialArpGS* serial        = nullptr;
    ArpGS::RadioMain* radioMain       = nullptr;
    ArpGS::BoardStatus* boardStatus   = nullptr;
    ArpGS::EthernetGS* ethernet       = nullptr;
    EthernetSniffer* ethernetSniffer  = nullptr;
    ArpGS::RadioPayload* radioPayload = nullptr;
};

/**
 * @brief Constructs and inserts the modules common to every GS into the given
 * DependencyManager.
 *
 * Does NOT insert a Hub or call manager.inject() -- callers must construct
 * and insert their own HubBase-derived Hub afterwards, then call inject()
 * themselves once everything (common + specific) has been added :)
 *
 * @return false if failed :(
 */
bool insertCommonModules(Boardcore::DependencyManager& manager,
                         const DipStatus& dipRead, CommonModules& outMods);

/**
 * @brief Starts the modules common to every GS
 * @param[out] initFatal set to true if main radio failed to start
 * @return false if any critical module failed to start.
 */
bool startCommonModules(CommonModules& mods, const DipStatus& dipRead,
                        Boardcore::PrintLogger& logger, bool& initFatal);

/**
 * @brief Logs which of the common radios/ethernet are actually present and
 * turns on the corresponding status LEDs.
 */
void reportCommonStatus(ArpGS::BoardStatus* boardStatus,
                        Boardcore::PrintLogger& logger);

/**
 * @brief Checks that the dip switch's ARP bit matches what this binary was
 * built to be
 *
 * @param isItArp what this compiled binary is (true = arp-gs-entry, false =
 * radio-gs-entry)
 */
bool checkDipMatchesBinary(const DipStatus& dipRead, bool isItArp,
                           Boardcore::PrintLogger& logger);

/// Used once init succeedes
void idleLoop();

/// Blinks the red LED at 5Hz forever
void errorLoop();

}  // namespace Groundstation

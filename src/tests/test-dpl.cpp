#include <miosix.h>
#include <utils/catch.hpp>

#include <skyward-boardcore/src/shared/utils/EventSniffer.h>
#include "DeathStack/DeploymentController/Deployment.h"
#include "DeathStack/Events.h"
#include "DeathStack/Topics.h"
#include "PinObserver.h"

using miosix::Thread;
using namespace DeathStackBoard;

void onEventReceived(uint8_t event, uint8_t topic)
{
    TRACE("[%.3f] %s on %s\n", (miosix::getTick() / 1000.0f),
          getEventString(event).c_str(), getTopicString(topic).c_str());
}

int main()
{
    DeploymentController* dpl = new DeploymentController();
    dpl->start();
    sEventBroker->start();

    EventSniffer* sniffer =
        new EventSniffer(*sEventBroker, TOPIC_LIST, onEventReceived);

    Thread::sleep(1000);

    printf("\n Init ok\n\n");
    while (true)
    {
        printf("o - nc open\n");
        printf("d - cut drogue\n");
        printf("m - cut main\n");
        printf("x - detach\n");

        char c = getchar();

        switch (c)
        {
            case ('o'):
                sEventBroker->post({EV_NC_OPEN}, TOPIC_DEPLOYMENT);
                break;
            case ('d'):
                sEventBroker->post({EV_CUT_MAIN}, TOPIC_DEPLOYMENT);
                break;
            case ('m'):
                sEventBroker->post({EV_CUT_DROGUE}, TOPIC_DEPLOYMENT);
                break;
            case ('x'):
                sEventBroker->post({EV_NC_DETACHED}, TOPIC_DEPLOYMENT);
                break;
            default:
                break;
        }
    }

    // EV_NC_OPEN EV_CUT_DROGUE EV_CUT_MAIN EV_NC_DETACHED
}
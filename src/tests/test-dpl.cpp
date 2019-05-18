#include <miosix.h>
#include <utils/catch.hpp>

#include "DeathStack/DeploymentController/Deployment.h"
#include "DeathStack/Events.h"
#include "DeathStack/Topics.h"
#include "PinObserver.h"
#include <skyward-boardcore/src/shared/utils/EventSniffer.h>

using miosix::Thread;
using namespace DeathStackBoard;

int main()
{
    DeploymentController* dpl = new DeploymentController();
    dpl->start();
    sEventBroker->start();

    EventSniffer* sniffer = new EventSniffer(*sEventBroker, 
                                              getEventString, 
                                              getTopicString);

    Thread::sleep(1000);

    printf("\n Init ok\n\n");
    while(true)
    {
        printf("o - nc open\n");
        printf("d - cut drogue\n");
        printf("m - cut main\n");
        printf("x - detach\n");

        char c = getchar();

        switch(c)
        {
            case('o'):
                sEventBroker->post({EV_NC_OPEN}, TOPIC_DEPLOYMENT);
                break;
            case('d'):
                sEventBroker->post({EV_CUT_MAIN}, TOPIC_DEPLOYMENT);
                break;
            case('m'):
                sEventBroker->post({EV_CUT_DROGUE}, TOPIC_DEPLOYMENT);
                break;
            case('x'):
                sEventBroker->post({EV_NC_DETACHED}, TOPIC_DEPLOYMENT);
                break;
            default:
                break;
        }

    }

    //EV_NC_OPEN EV_CUT_DROGUE EV_CUT_MAIN EV_NC_DETACHED
}
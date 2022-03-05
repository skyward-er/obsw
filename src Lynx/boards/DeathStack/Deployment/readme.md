```mermaid
stateDiagram-v2
    [*] --> idle
    idle --> nosecone_ejection : EV_NC_OPEN
    idle --> cutting_primary : EV_CUT_DROGUE
    idle --> test_cutting_primary : EV_TEST_CUT_PRIMARY
    idle --> test_cutting_backup : EV_TEST_CUT_BACKUP
    nosecone_ejection --> idle : EV_NC_DETACHED
    nosecone_ejection --> idle : EV_NC_OPEN_TIMEOUT
    cutting_primary --> cutting_backup : EV_CUTTING_TIMEOUT
    cutting_backup --> idle : EV_CUTTING_TIMEOUT
    test_cutting_primary --> idle : EV_CUTTING_TIMEOUT
    test_cutting_backup --> idle : EV_CUTTING_TIMEOUT
```

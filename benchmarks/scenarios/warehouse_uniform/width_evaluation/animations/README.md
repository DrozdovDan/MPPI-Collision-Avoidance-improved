# Recorded width-weighted K-route motion

Actual saved poses; playback compresses simulated time. Dashed lines are the selected keep-right anchors; solid traces are actual motion. Examples select the first success for each map/traffic-pattern, or the first saved run when no success exists. All tasks remain in the reported denominator.

For a full-length, 1× comparison with every recorded 0.1 s step, see [RRT versus both K-route policies on warehouse_s13_v02](../full_replays/warehouse_s13_v02/README.md). It shows all 32 planned paths and executed trails in three synchronized columns, with agent IDs and close-ups.

| Map | Traffic pattern | Outcome | GIF |
| --- | --- | --- | --- |
| warehouse | distributed | success | [animation](warehouse_s16_v03__k_routes_width__p42__c42__success.gif) |
| warehouse | opposing_bottleneck | deadlock | [animation](warehouse_s02_v01__k_routes_width__p42__c42__deadlock.gif) |
| warehouse | intersection | deadlock | [animation](warehouse_s03_v01__k_routes_width__p42__c42__deadlock.gif) |
| warehouse | symmetric_swap | success | [animation](warehouse_s04_v02__k_routes_width__p42__c42__success.gif) |
| warehouse | competing_bottlenecks | deadlock | [animation](warehouse_s05_v01__k_routes_width__p42__c42__deadlock.gif) |
| warehouse2 | distributed | success | [animation](warehouse2_s01_v03__k_routes_width__p42__c42__success.gif) |
| warehouse2 | opposing_bottleneck | deadlock | [animation](warehouse2_s02_v01__k_routes_width__p42__c42__deadlock.gif) |
| warehouse2 | intersection | success | [animation](warehouse2_s03_v03__k_routes_width__p42__c42__success.gif) |
| warehouse2 | symmetric_swap | success | [animation](warehouse2_s04_v05__k_routes_width__p42__c42__success.gif) |
| warehouse2 | competing_bottlenecks | deadlock | [animation](warehouse2_s05_v01__k_routes_width__p42__c42__deadlock.gif) |

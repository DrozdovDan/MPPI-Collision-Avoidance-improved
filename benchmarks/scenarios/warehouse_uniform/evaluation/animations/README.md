# Recorded warehouse evaluation motion

These GIFs use actual saved states. The displayed timestamps are simulated seconds; playback compresses the observation into at most eight seconds. Agent footprints and headings retain the measured poses. The right panel zooms into the terminal blockage when available.

| Map | Method | Traffic pattern | Outcome | Observed simulated seconds | GIF |
| --- | --- | --- | --- | ---: | --- |
| warehouse | rrt | intersection | success | 80.9 | [animation](warehouse_s03_v01__rrt__p42__c42__success.gif) |
| warehouse | k_routes | distributed | deadlock | 118.0 | [animation](warehouse_s01_v01__k_routes__p42__c42__deadlock.gif) |
| warehouse | rrt | distributed | success | 111.1 | [animation](warehouse_s06_v03__rrt__p42__c42__success.gif) |
| warehouse | k_routes | opposing_bottleneck | deadlock | 74.0 | [animation](warehouse_s02_v01__k_routes__p42__c42__deadlock.gif) |
| warehouse | rrt | symmetric_swap | success | 87.6 | [animation](warehouse_s04_v02__rrt__p42__c42__success.gif) |
| warehouse | k_routes | competing_bottlenecks | deadlock | 95.0 | [animation](warehouse_s05_v01__k_routes__p42__c42__deadlock.gif) |
| warehouse2 | rrt | distributed | success | 93.8 | [animation](warehouse2_s01_v03__rrt__p42__c42__success.gif) |
| warehouse2 | k_routes | symmetric_swap | success | 186.3 | [animation](warehouse2_s04_v05__k_routes__p42__c42__success.gif) |
| warehouse2 | k_routes | opposing_bottleneck | deadlock | 112.0 | [animation](warehouse2_s02_v01__k_routes__p42__c42__deadlock.gif) |
| warehouse2 | rrt | intersection | success | 65.5 | [animation](warehouse2_s03_v02__rrt__p42__c42__success.gif) |
| warehouse2 | rrt | symmetric_swap | success | 104.5 | [animation](warehouse2_s04_v03__rrt__p42__c42__success.gif) |
| warehouse2 | k_routes | competing_bottlenecks | deadlock | 121.0 | [animation](warehouse2_s05_v01__k_routes__p42__c42__deadlock.gif) |
| warehouse2 | k_routes | distributed | deadlock | 184.0 | [animation](warehouse2_s01_v01__k_routes__p42__c42__deadlock.gif) |

# Restricted Dual Kinova demonstration wheel license

Copyright (c) 2026 Chuizheng Kong. All rights reserved. Proprietary.

## Limited permission: demonstration only

The copyright holder grants the recipient a personal, non-exclusive,
non-transferable, non-sublicensable permission, free of charge, to install and
execute the supplied geo_kin Dual Kinova 3 binary wheel solely as a dependency
of the supplied vr_robosuite Dual Kinova simulation demonstration environment
(the Authorized Environment), on any compatible computer, with its supplied
robot models and Robotiq or Psyonic Ability grippers. This software is provided ONLY FOR DEMO PURPOSES.
No permission is granted for standalone solver use, other applications,
environments, robots, commercial services, production, deployment, or real
robot / physical actuator control. Any additional use requires prior written
permission from the copyright holder. Source code is not licensed here.

## Access tiers

The free demonstration grant enables POSE MODE ONLY. TCP mode is unavailable
under that grant. A valid, verified developer license covering Dual Kinova and
the requested hand takes priority, including when this wheel was installed as
a demo: developer sessions have full pose/TCP access and no demonstration time
limit. Developer and other separately licensed full-product use is governed by
its separate license, not by this free demonstration grant.

Installation of the bundled demo wheel is an explicit provisioner action;
normal application dependency installation does not select or replace a wheel.

## One-minute demonstration runs

Each unlicensed demo run permits 60 seconds of elapsed monotonic time, starting at the
first `solve()` call. All free demo solver sessions in the same Python process
share that deadline; verified developer sessions are exempt. At expiry, new solves, session construction, resets, and
robot FK calls fail with `DemoExpiredError`; solver output stops. Resetting,
recreating, or re-importing the solver does not renew the deadline. There is no
automatic cooldown or resume. Close and restart the demo application to begin
another permitted demonstration. Setup and the startup banner do not consume
the one-minute budget. The banner animates once per process over a randomly
selected total duration of 0.5 to 1 second, subject to system scheduling and
output-device delays.

Manual restarts for additional demonstrations are permitted. Automated process
rotation, time-limit bypass, or chaining runs to provide continuous solver use
is prohibited without prior written permission. The timer is a practical demo
restriction, not a security boundary: a recipient controlling the host can
restart processes or alter software. Banner animation is presentation only.

## No redistribution or transfer

You must not redistribute, publish, upload, mirror, sell, rent, lend, sublicense,
share, or otherwise provide the wheel, its native libraries, extracted solver
code, or modified versions to another person or organization. This includes
package indexes, repositories, release assets, containers, virtual machine
images, application bundles, and network services exposing solver access.
Only the copyright holder or a distributor expressly authorized in writing may
supply copies. Recipients may download or clone the official demonstration
bundle, including its wheel, and install it on their own compatible computers
for the permitted demo use. They may not republish or distribute those copies.
You may make installation and private backup copies only as needed for your own authorized demonstration use; preserve this notice and
restrict access to those copies. Access to a download is not redistribution
permission. The surrounding application's license does not license this wheel.

Except where applicable law prohibits such restrictions, you must not modify,
reverse engineer, decompile, disassemble, extract, or repurpose the proprietary
solver, remove notices, bypass environment or license checks, or assist others
to do so. All rights not expressly granted are reserved.

## Simulation safety precautions

This is experimental demonstration software, not a validated robot controller
or safety component. Use simulation only. Do not connect its outputs, directly
or indirectly, to physical robots, grippers, actuators, or safety systems.
Keep hardware interfaces and actuation disabled and disconnected. Do not rely
on collision filtering, joint limits, tracking, model geometry, or solver output
for personnel or equipment safety: these may be incomplete or incorrect.

Supervise demonstrations. Check model configuration, coordinate frames, units,
joint ranges, input validity, and output plausibility before use. Start paused
or at low simulated speed; maintain a stop control. Stop on tracking loss,
non-finite values, unexpected motion, or simulation faults. Keep the physical
VR play area clear and use appropriate headset boundaries. Do not use in
medical, life-critical, hazardous, or other safety-critical applications.
These precautions are not a safety certification or an exhaustive risk assessment.

## Warranty, responsibility, and termination

TO THE MAXIMUM EXTENT PERMITTED BY APPLICABLE LAW, THE SOFTWARE IS PROVIDED
"AS IS" AND "AS AVAILABLE", WITHOUT WARRANTIES OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, NON-INFRINGEMENT,
ACCURACY, RELIABILITY, OR SAFETY. THE COPYRIGHT HOLDER AND CONTRIBUTORS SHALL NOT
BE LIABLE FOR ANY CLAIM, DAMAGE, INJURY, LOSS, OR OTHER LIABILITY ARISING FROM
THE SOFTWARE OR ITS USE, WHETHER IN CONTRACT, TORT, OR OTHERWISE, TO THE EXTENT
PERMITTED BY LAW. The recipient is responsible for evaluating and supervising
permitted use. No provision excludes rights or liabilities that applicable law
does not permit to be excluded.

Permission terminates automatically on breach. On termination, stop use and
delete copies under your control, except copies required by law to be retained.
Included third-party components retain their own licenses and notices; these
terms do not restrict rights those licenses independently grant. Other geo_kin
products retain their separate license requirements.

These terms apply to copies supplied under this notice. They do not purport to
retroactively change permissions attached to previously supplied copies.
No machine lock, activation, or organization license file is required for this
demo product. The restrictions above are license conditions, not a technical
guarantee that the wheel cannot execute elsewhere. Different robot geometry
does not provide a security boundary or establish suitability for hardware use.

// M2 multi-mode firmware — STN mode implementation (Phase 4).
// See mode_stn.h for the contract. All STN behavior lives in the ELM core's
// STN extension (mode_elm.cpp); this file is the mode-manager adapter.

#include "mode_stn.h"
#include "mode_elm.h"

namespace stn {

void begin()
{
    elm::setStnMode(true);  // enable ST-command parsing first...
    elm::begin();           // ...then reset state (begin() keeps the flag)
}

void loop()
{
    elm::loop();  // shared pump: AT + ST + OBD hex + mm divert
}

void exit()
{
    elm::setStnMode(false);
    elm::exit();  // releases CAN0 / J1850 if held (idempotent)
}

} // namespace stn

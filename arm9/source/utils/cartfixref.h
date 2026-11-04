#pragma once

#include "common.h"

// Repair a cartridge by comparing it sector by sector with a verified .3ds of
// the same game and re-reading (with refresh) only the sectors that differ.
// `cart_path` must be the .3ds file on the [C:] GAMECART drive.
u32 FixCartWithReference(const char* cart_path);

// Create SD:/luma/payloads/hash/<product>_<titleid>.crh from a healthy cartridge.
u32 CreateCartRefHash(const char* cart_path);

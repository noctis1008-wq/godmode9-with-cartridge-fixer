#pragma once

void MyriaSysinfo(char* sysinfo_txt);

// The console's serial number (as on the label underneath, from SecureInfo)
// and model name, for telling consoles apart in logs. "<unknown>" if unreadable.
void GetConsoleIdentity(char* serial, size_t serial_size, char* model, size_t model_size);

#pragma once

void printResetReason( Print& serial );
void printEnvironment( Print& serial );
void printMemoryInfo( Print& serial );
void printNetworkInfo( Print& serial );

void reportEnvironmentJSON( JsonDocument& doc );
const char* reportEnvironmentString();
void reportMemoryInfoJSON( JsonDocument& doc );
const char* reportMemoryInfoString();
void reportNetworkInfoJSON( JsonDocument& doc );
const char* reportNetworkInfoString();
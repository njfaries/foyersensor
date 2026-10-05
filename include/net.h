// Wi-Fi, ntfy alerts, and the HTTP control endpoint.
#pragma once

// Starts Wi-Fi and the HTTP server. Does not block.
void netBegin();

// Keeps Wi-Fi connected and serves HTTP. Call every loop.
void netLoop();

// Queues an ntfy message. Returns at once; a task sends it.
void netNotify(const char *title, const char *message, int priority,
               const char *tags, bool withResetAction);

// True once after a network client asked for a reset.
bool netResetRequested();

// Supplies the current state text for the /status endpoint.
void netSetStatus(const char *stateText, bool alarmOn);

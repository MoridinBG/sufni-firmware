#ifndef LIVE_STREAM_CORE1_H
#define LIVE_STREAM_CORE1_H

#include <stdbool.h>

struct tcpserver;

bool live_stream_core1_can_accept_client(const struct tcpserver *server);
void live_stream_core1_reset(struct tcpserver *server);
bool live_stream_core1_process_rx(struct tcpserver *server);
void live_stream_core1_service(struct tcpserver *server);
void live_stream_core1_abort(struct tcpserver *server);

#endif // LIVE_STREAM_CORE1_H
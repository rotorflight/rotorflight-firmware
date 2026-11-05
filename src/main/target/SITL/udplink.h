/**
 * Copyright (c) 2017 cs8425
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the MIT license.
 */

#ifndef __UDPLINK_H
#define __UDPLINK_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#if defined(_WIN32) || defined(__MINGW32__)
// Trim down windows.h (pulled in transitively via winsock2.h) so it does not
// define the legacy serial-comm BAUD_* macros (from winbase.h), which clash
// with the BAUD_* enum values declared in io/serial.h.
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOCOMM
#define NOCOMM
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32) || defined(__MINGW32__)
typedef SOCKET udp_socket_t;
#else
typedef int udp_socket_t;
#endif

typedef struct {
    udp_socket_t fd;
    struct sockaddr_in si;
    struct sockaddr_in recv;
    int port;
    char* addr;
    bool isServer;
} udpLink_t;

int udpInit(udpLink_t* link, const char* addr, int port, bool isServer);
int udpRecv(udpLink_t* link, void* data, size_t size, uint32_t timeout_ms);
int udpSend(udpLink_t* link, const void* data, size_t size);

#ifdef __cplusplus
} // extern "C"
#endif

#endif

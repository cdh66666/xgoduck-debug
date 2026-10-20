/*
    This file is part of the Arduino_RouterBridge library.

    Copyright (C) Arduino s.r.l. and/or its affiliated companies

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.

*/

#pragma once

#ifndef ROUTER_BRIDGE_H
#define ROUTER_BRIDGE_H

#define RESET_METHOD "$/reset"
#define BIND_METHOD "$/register"
#define GET_VERSION_METHOD "$/version"
#define SET_BUF_METHOD "$/setMaxMsgSize"

//#define BRIDGE_ERROR "$/bridgeLog"

#define UPDATE_THREAD_STACK_SIZE    4096
#define UPDATE_THREAD_PRIORITY      5

#define DEFAULT_SERIAL_BAUD         115200
#define BRIDGE_RPC_BUFFER_SIZE      DEFAULT_RPC_BUFFER_SIZE

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <Arduino_RPClite.h>

#include <utility>


void updateEntryPoint(void *, void *, void *);

template<typename... Args>
class RpcCall {

    RpcError error;

    void setError(int code, MsgPack::str_t text) {
        k_mutex_lock(&call_mutex, K_FOREVER);
        error.code = code;
        error.traceback = std::move(text);
        k_mutex_unlock(&call_mutex);
    }

public:

    RpcCall(const MsgPack::str_t& m, RPCClient* c, struct k_mutex* rm, struct k_mutex* wm, Args&&... args): method(m), client(c), read_mutex(rm), write_mutex(wm), callback_params(std::forward_as_tuple(std::forward<Args>(args)...)) {
        k_mutex_init(&call_mutex);
        setError(GENERIC_ERR, "This call is not yet executed");
    }

    bool isError() {
        k_mutex_lock(&call_mutex, K_FOREVER);
        const bool out = error.code > NO_ERR;
        k_mutex_unlock(&call_mutex);
        return out;
    }

    int getErrorCode() {
        k_mutex_lock(&call_mutex, K_FOREVER);
        const int out = error.code;
        k_mutex_unlock(&call_mutex);
        return out;
    }

    MsgPack::str_t getErrorMessage() {
        k_mutex_lock(&call_mutex, K_FOREVER);
        MsgPack::str_t out = error.traceback;
        k_mutex_unlock(&call_mutex);
        return out;
    }

    template<typename RType> bool result(RType& result) {

        if (!atomic_cas(&_executed, 0, 1)){
            // this thread lost the race
            setError(GENERIC_ERR, "This call is no longer available");
            return false;
        }

        while (true) {
            if (k_mutex_lock(write_mutex, K_MSEC(10)) == 0) {
                std::apply([this](const auto&... elems) {
                    client->send_rpc(method, msg_id_wait, elems...);
                }, callback_params);
                k_mutex_unlock(write_mutex);
                break;
            } else {
                k_yield();
            }
        }

        while(true) {
            if (k_mutex_lock(read_mutex, K_MSEC(10)) == 0 ) {
                RpcError temp_err;
                if (client->get_response(msg_id_wait, result, temp_err)) {
                    k_mutex_unlock(read_mutex);
                    // if (error.code == PARSING_ERR) {
                    //     k_mutex_lock(write_mutex, K_FOREVER);
                    //     client->notify(BRIDGE_ERROR, error.traceback);
                    //     k_mutex_unlock(write_mutex);
                    // }
                    setError(temp_err.code, temp_err.traceback);
                    break;
                }
                k_mutex_unlock(read_mutex);
                k_msleep(1);
            } else {
                k_yield();
            }
        }

        return !isError();
    }

    bool result() {
        MsgPack::object::nil_t nil;
        return result(nil);
    }

    ~RpcCall(){
        result();
    }

    operator bool() {
        return result();
    }

private:
    uint32_t msg_id_wait{};
    atomic_t _executed = ATOMIC_INIT(0);;

    MsgPack::str_t method;
    RPCClient* client;
    struct k_mutex* read_mutex;
    struct k_mutex* write_mutex;
    struct k_mutex call_mutex{};
    std::tuple<Args...> callback_params;
};

class BridgeClass {

    RPCClient* client = nullptr;
    RPCServer* server = nullptr;
    HardwareSerial* serial_ptr = nullptr;
    ITransport* transport = nullptr;

    struct k_mutex read_mutex{};
    struct k_mutex write_mutex{};
    struct k_mutex bridge_mutex{};
    struct k_mutex route_mutex{};

    k_tid_t upd_tid{};
    k_thread_stack_t *upd_stack_area{};
    struct k_thread upd_thread_data{};

    bool started = false;
    bool prepared = false;
    bool preparationFailed = false;
    bool resetConfirmed = false;

public:

    explicit BridgeClass(HardwareSerial& serial) {
        serial_ptr = &serial;
    }

    operator bool() {
        return is_started();
    }

    bool is_started() {
        k_mutex_lock(&bridge_mutex, K_FOREVER);
        bool out = started;
        k_mutex_unlock(&bridge_mutex);
        return out;
    }

    // Initialize the bridge
    bool begin(unsigned long baud=DEFAULT_SERIAL_BAUD) {
        k_mutex_init(&read_mutex);
        k_mutex_init(&write_mutex);
        k_mutex_init(&bridge_mutex);
        k_mutex_init(&route_mutex);

        if (is_started()) return true;

        k_mutex_lock(&bridge_mutex, K_FOREVER);

        serial_ptr->begin(baud);
        while (!*serial_ptr){
            k_yield();
        }
#if defined(ARDUINO_UNO_Q) 
        // Flush broken RPCs from the previous run on hardware serial.
        serial_ptr->write("MCU starting RPC Bridge communication");
#endif
        transport = new SerialTransport(*serial_ptr);

        client = new RPCClient(*transport);
        server = new RPCServer(*transport);

        upd_stack_area = k_thread_stack_alloc(UPDATE_THREAD_STACK_SIZE, 0);
        upd_tid = k_thread_create(&upd_thread_data, upd_stack_area,
                                UPDATE_THREAD_STACK_SIZE,
                                updateEntryPoint,
                                NULL, NULL, NULL,
                                UPDATE_THREAD_PRIORITY, 0, K_NO_WAIT);
        k_thread_name_set(upd_tid, "bridge");
        prepared = true;

        bool res = false;
        started = call(RESET_METHOD).result(res) && res;
        notify(SET_BUF_METHOD, BRIDGE_RPC_BUFFER_SIZE);     // no effect if Router is not exposing SET_BUF_METHOD
        k_mutex_unlock(&bridge_mutex);
        return res;
    }

    // Single startup owner only. Allocate transport/decoder/update thread once;
    // retries below send only a new tracked Router maintenance request.
    // Ordinary Bridge.call/user commands retain their original no-retry API.
    bool beginBounded(unsigned long baud = DEFAULT_SERIAL_BAUD, uint32_t timeoutMs = 200) {
        if (!timeoutMs || timeoutMs > 200 || preparationFailed) return false;
        if (!prepared) {
            k_mutex_init(&read_mutex); k_mutex_init(&write_mutex);
            k_mutex_init(&bridge_mutex); k_mutex_init(&route_mutex);
            serial_ptr->begin(baud);
            transport = new SerialTransport(*serial_ptr);
            client = new RPCClient(*transport);
            server = new RPCServer(*transport);
            upd_stack_area = k_thread_stack_alloc(UPDATE_THREAD_STACK_SIZE, 0);
            if (!transport || !client || !server || !upd_stack_area) {
                preparationFailed = true;
                return false;
            }
            upd_tid = k_thread_create(&upd_thread_data, upd_stack_area, UPDATE_THREAD_STACK_SIZE,
                                     updateEntryPoint, NULL, NULL, NULL, UPDATE_THREAD_PRIORITY, 0, K_NO_WAIT);
            if (!upd_tid) { preparationFailed = true; return false; }
            k_thread_name_set(upd_tid, "bridge");
            prepared = true;
        }
        if (k_mutex_lock(&bridge_mutex, K_NO_WAIT) != 0) return false;
        const bool alreadyStarted = started;
        k_mutex_unlock(&bridge_mutex);
        if (alreadyStarted) return true;
        if (!resetConfirmed) resetConfirmed = maintenanceCall(RESET_METHOD, timeoutMs);
        if (!resetConfirmed || k_mutex_lock(&bridge_mutex, K_NO_WAIT) != 0) return false;
        started = true;
        k_mutex_unlock(&bridge_mutex);
        // Optional Router receive-size hint: one nonblocking whole-frame write,
        // never extend the handshake deadline waiting for a notification.
        MsgPack::Packer hint;
        hint.serialize(MsgPack::arr_size_t(3), NOTIFY_MSG, MsgPack::str_t(SET_BUF_METHOD),
                       MsgPack::arr_size_t(1), BRIDGE_RPC_BUFFER_SIZE);
        if (k_mutex_lock(&write_mutex, K_NO_WAIT) == 0) {
            if (serial_ptr->availableForWrite() >= int(hint.size()))
                serial_ptr->write(hint.data(), hint.size());
            k_mutex_unlock(&write_mutex);
        }
        return true;
    }

    template<typename F>
    bool bindLocalOnce(const MsgPack::str_t& name, F&& func) {
        // Local callback availability only, NEVER proof of Router ownership.
        // Router 0.5.3 returns code 5 on duplicate registration, even for the
        // same peer; a lost first ACK must not strand local binding or reports.
        if (!prepared) return false;
        if (k_mutex_lock(&bridge_mutex, K_NO_WAIT) != 0) return false;
        const bool out = started && (server->isBound(name) || server->bind(name, std::forward<F>(func)));
        k_mutex_unlock(&bridge_mutex);
        return out;
    }

    // Only refresh an already bound local method. No bind(), reset(), controller
    // callback, target replay or public RPC registration on another connection.
    bool registerOnly(const MsgPack::str_t& name, uint32_t timeoutMs = 200) {
        if (!client || !server || !timeoutMs || timeoutMs > 200) return false;
        if (k_mutex_lock(&bridge_mutex, K_NO_WAIT) != 0) return false;
        const bool permitted = started && server->isBound(name);
        k_mutex_unlock(&bridge_mutex);
        if (!permitted) return false;
        return maintenanceCall(BIND_METHOD, timeoutMs, name);
    }

    bool getRouterVersion(MsgPack::str_t& version) {
        return call(GET_VERSION_METHOD).result(version);
    }

    template<typename F>
    bool provide(const MsgPack::str_t& name, F&& func) {
        k_mutex_lock(&bridge_mutex, K_FOREVER);
        bool res;
        bool out = call(BIND_METHOD, name).result(res) && res && server->bind(name, func);
        k_mutex_unlock(&bridge_mutex);
        return out;
    }

    template<typename F>
    bool provide_safe(const MsgPack::str_t& name, F&& func) {
        k_mutex_lock(&bridge_mutex, K_FOREVER);
        bool res;
        bool out = call(BIND_METHOD, name).result(res) && res && server->bind(name, func, "__safe__");
        k_mutex_unlock(&bridge_mutex);
        return out;
    }

    void update() {

        // Lock read mutex
        if (k_mutex_lock(&read_mutex, K_MSEC(10)) != 0 ) return;

        RPCRequest<BRIDGE_RPC_BUFFER_SIZE> req;
        if (!server->get_rpc(req)) {
            k_mutex_unlock(&read_mutex);
            k_msleep(1);
            return;
        }

        k_mutex_unlock(&read_mutex);

        server->process_request(req);

        // Lock write mutex
        while (true) {

            if (k_mutex_lock(&write_mutex, K_MSEC(10)) == 0){
                server->send_response(req);
                k_mutex_unlock(&write_mutex);
                break;
            } else {
                k_yield();
            }

        }

    }

    template<typename... Args>
    RpcCall<Args...> call(const MsgPack::str_t& method, Args&&... args) {
       return RpcCall<Args...>(method, client, &read_mutex, &write_mutex, std::forward<Args>(args)...);
    }

    template<typename... Args>
    void notify(const MsgPack::str_t method, Args&&... args)  {
        while (true) {
            if (k_mutex_lock(&write_mutex, K_MSEC(10)) == 0) {
                client->notify(method, std::forward<Args>(args)...);
                k_mutex_unlock(&write_mutex);
                break;
            }
            k_yield();
        }
    }

private:

    // Odd IDs are private to initialization/route maintenance. Each timed-out
    // or completed request is retired; its late ACK cannot complete a new one.
    template<typename... Args>
    bool maintenanceCall(const MsgPack::str_t& method, uint32_t timeoutMs, Args&&... args) {
        if (!client || !server || !timeoutMs || timeoutMs > 200) return false;
        if (k_mutex_lock(&route_mutex, K_NO_WAIT) != 0) return false;
        struct Unlock { k_mutex* mutex; ~Unlock() { k_mutex_unlock(mutex); } } unlock{&route_mutex};
        const uint32_t start = k_uptime_get_32();
        MsgPack::Packer packed; uint32_t id;
        client->prepare_maintenance_rpc(packed, id, method, std::forward<Args>(args)...);
        struct Retire { RPCClient* client; uint32_t id;
            ~Retire() { client->retire_route_registration(id); } } retire{client, id};
        bool sent = false;
        while (uint32_t(k_uptime_get_32() - start) < timeoutMs) {
            if (!sent && k_mutex_lock(&write_mutex, K_NO_WAIT) == 0) {
                if (serial_ptr->availableForWrite() >= int(packed.size())) {
                    sent = serial_ptr->write(packed.data(), packed.size()) == packed.size();
                    k_mutex_unlock(&write_mutex);
                    if (!sent) return false;
                } else k_mutex_unlock(&write_mutex);
            }
            if (sent && k_mutex_lock(&read_mutex, K_NO_WAIT) == 0) {
                bool accepted = false; RpcError error;
                const bool received = client->get_response(id, accepted, error);
                k_mutex_unlock(&read_mutex);
                if (received) return error.code == NO_ERR && accepted;
            }
            k_msleep(1);
        }
        return false;
    }

    void update_safe() {

        // Lock read mutex
        if (k_mutex_lock(&read_mutex, K_MSEC(10)) != 0 ) return;

        RPCRequest<BRIDGE_RPC_BUFFER_SIZE> req;
        if (!server->get_rpc(req, "__safe__")) {
            k_mutex_unlock(&read_mutex);
            k_msleep(1);
            return;
        }

        k_mutex_unlock(&read_mutex);

        server->process_request(req);

        // Lock write mutex
        while (true) {

            if (k_mutex_lock(&write_mutex, K_MSEC(10)) == 0){
                server->send_response(req);
                k_mutex_unlock(&write_mutex);
                break;
            } else {
                k_yield();
            }

        }

    }

    friend class BridgeClassUpdater;

};

extern BridgeClass Bridge;

class BridgeClassUpdater {
public:
    static void safeUpdate(BridgeClass* bridge) {
        if (*bridge) {
            bridge->update_safe();
        }
    }

private:
    BridgeClassUpdater() = delete; // prevents instantiation
};

inline void updateEntryPoint(void *, void *, void *){
    while (true) {
        if (Bridge) {
            Bridge.update();
            k_yield();
        } else {
            // This thread has priority 5; sketch main may have priority 14.
            // k_yield() alone only helps peers at this priority, so a not-yet
            // started Bridge must block to let main finish the RESET handshake.
            k_msleep(1);
        }
    }
}

static void safeUpdate(){
    BridgeClassUpdater::safeUpdate(&Bridge);
}

// leave as is
void __attribute__((weak)) __loopHook(void){
    k_yield();
    safeUpdate();
}

#endif // ROUTER_BRIDGE_H

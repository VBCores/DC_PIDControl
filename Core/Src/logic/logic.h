#pragma once

#include <memory>

#include "fdcan.h"

#include <motors/dc/dc.h>

#include "cyphal/cyphal.h"
#include "cyphal/subscriptions/subscription.h"

#include <types/uavcan/_register/Access_1_0.h>
#include <types/uavcan/_register/List_1_0.h>
#include <types/uavcan/node/GetInfo_1_0.h>
#include "types/uavcan/node/Heartbeat_1_0.h"
#include <types/uavcan/si/unit/angular_velocity/Scalar_1_0.h>
#include <types/uavcan/si/unit/angle/Scalar_1_0.h>

#include <types/voltbro/echo/echo_service_1_0.h>

TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)
TYPE_ALIAS(UString, uavcan_primitive_String_1_0)
TYPE_ALIAS(AngularVelocityScalar, uavcan_si_unit_angular_velocity_Scalar_1_0)
TYPE_ALIAS(AngleScalar, uavcan_si_unit_angle_Scalar_1_0)

TYPE_ALIAS(RegisterListRequest, uavcan_register_List_Request_1_0)
TYPE_ALIAS(RegisterListResponse, uavcan_register_List_Response_1_0)

TYPE_ALIAS(RegisterAccessRequest, uavcan_register_Access_Request_1_0)
TYPE_ALIAS(RegisterAccessResponse, uavcan_register_Access_Response_1_0)

TYPE_ALIAS(NodeInfoRequest, uavcan_node_GetInfo_Request_1_0)
TYPE_ALIAS(NodeInfoResponse, uavcan_node_GetInfo_Response_1_0)

TYPE_ALIAS(EchoRequest, voltbro_echo_echo_service_Request_1_0)
TYPE_ALIAS(EchoResponse, voltbro_echo_echo_service_Response_1_0)

#define ECHO_SERVICE_ID 471U
#define SELF_DIAG_PORT 176

class RegisterListReader : public AbstractSubscription<RegisterListRequest> {
public:
    RegisterListReader(InterfacePtr interface): AbstractSubscription<RegisterListRequest>(
            interface,
            uavcan_register_List_1_0_FIXED_PORT_ID_
    ) {};
    void handler(const RegisterListRequest::Type&, CanardRxTransfer*) override;
};

class RegisterAccessReader : public AbstractSubscription<RegisterAccessRequest> {
public:
    RegisterAccessReader(InterfacePtr interface): AbstractSubscription<RegisterAccessRequest>(
            interface,
            uavcan_register_Access_1_0_FIXED_PORT_ID_
    ) {};
    void handler(const RegisterAccessRequest::Type&, CanardRxTransfer*) override;
};

class NodeInfoReader : public AbstractSubscription<NodeInfoRequest> {
public:
    NodeInfoReader(InterfacePtr interface): AbstractSubscription<NodeInfoRequest>(
            interface,
            uavcan_node_GetInfo_1_0_FIXED_PORT_ID_
    ) {};
    void handler(const NodeInfoRequest::Type&, CanardRxTransfer*) override;
};

class EchoReader : public AbstractSubscription<EchoRequest> {
public:
    EchoReader(InterfacePtr interface): AbstractSubscription<EchoRequest>(
            interface,
            ECHO_SERVICE_ID
    ) {};
    void handler(const EchoRequest::Type&, CanardRxTransfer*) override;
};

// Utils
uint64_t micros_64();
void error_handler();
void run_self_diagnostic();

// Communication
std::shared_ptr<CyphalInterface> get_interface();
CanardNodeID get_node_id();
void send_diagnostic(char* string);
void send_angular_vel(float angular_velocity);
void send_angle(float angle);
bool is_cyphal_on();
void reporting_loop(uint32_t millis, DCMotorController&);
void setup_cyphal();

// Motor
DCMotorController& get_motor();

#define VCS_REVISION_ID 1
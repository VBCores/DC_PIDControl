#pragma once

#include <memory>

#include "fdcan.h"

#include <voltbro/motors/dc/dc.h>

#include "cyphal/cyphal.h"
#include "cyphal/subscriptions/subscription.h"

#include <types/uavcan/_register/Access_1_0.h>
#include <types/uavcan/_register/List_1_0.h>
#include <types/uavcan/node/GetInfo_1_0.h>
#include "types/uavcan/node/Heartbeat_1_0.h"
#include <types/uavcan/si/unit/angular_velocity/Scalar_1_0.h>
#include <types/uavcan/si/unit/angle/Scalar_1_0.h>

#include <types/voltbro/config/dc/get_1_0.h>
#include <types/voltbro/config/dc/set_1_0.h>

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

TYPE_ALIAS(GetConfigRequest, voltbro_config_dc_get_Request_1_0)
TYPE_ALIAS(GetConfigResponse, voltbro_config_dc_get_Response_1_0)
TYPE_ALIAS(SetConfigRequest, voltbro_config_dc_set_Request_1_0)
TYPE_ALIAS(SetConfigResponse, voltbro_config_dc_set_Response_1_0)
TYPE_ALIAS(PIDConfigMessage, voltbro_config_dc_pid_config_1_0)
TYPE_ALIAS(PIDReportMessage, voltbro_config_dc_pid_report_1_0)

#define SPEED_TARGET_PORT 555U
#define SELF_DIAG_PORT 176U
// TODO: define next 2 using network_tools
#define GET_CONFIG_SERVICE_ID 222U
#define SET_CONFIG_SERVICE_ID 223U

class GetConfigReader : public AbstractSubscription<GetConfigRequest> {
public:
    GetConfigReader(InterfacePtr interface): AbstractSubscription<GetConfigRequest>(
            interface,
            GET_CONFIG_SERVICE_ID,
            CanardTransferKindRequest
    ) {};
    void handler(const GetConfigRequest::Type&, CanardRxTransfer*) override;
};

class SetConfigReader : public AbstractSubscription<SetConfigRequest> {
public:
    SetConfigReader(InterfacePtr interface): AbstractSubscription<SetConfigRequest>(
            interface,
            SET_CONFIG_SERVICE_ID,
            CanardTransferKindRequest
    ) {};
    void handler(const SetConfigRequest::Type&, CanardRxTransfer*) override;
};


class SpeedTargetReader : public AbstractSubscription<AngularVelocityScalar> {
public:
    SpeedTargetReader(InterfacePtr interface): AbstractSubscription<AngularVelocityScalar>(
            interface,
            SPEED_TARGET_PORT,
            CanardTransferKindMessage
    ) {};
    void handler(const AngularVelocityScalar::Type&, CanardRxTransfer*) override;
};

class RegisterListReader : public AbstractSubscription<RegisterListRequest> {
public:
    RegisterListReader(InterfacePtr interface): AbstractSubscription<RegisterListRequest>(
            interface,
            uavcan_register_List_1_0_FIXED_PORT_ID_,
            CanardTransferKindRequest
    ) {};
    void handler(const RegisterListRequest::Type&, CanardRxTransfer*) override;
};

class RegisterAccessReader : public AbstractSubscription<RegisterAccessRequest> {
public:
    RegisterAccessReader(InterfacePtr interface): AbstractSubscription<RegisterAccessRequest>(
            interface,
            uavcan_register_Access_1_0_FIXED_PORT_ID_,
            CanardTransferKindRequest
    ) {};
    void handler(const RegisterAccessRequest::Type&, CanardRxTransfer*) override;
};

class NodeInfoReader : public AbstractSubscription<NodeInfoRequest> {
public:
    NodeInfoReader(InterfacePtr interface): AbstractSubscription<NodeInfoRequest>(
            interface,
            uavcan_node_GetInfo_1_0_FIXED_PORT_ID_,
            CanardTransferKindRequest
    ) {};
    void handler(const NodeInfoRequest::Type&, CanardRxTransfer*) override;
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

// EEPROM
void persist_pid();
void reload_pid();

#define VCS_REVISION_ID 1
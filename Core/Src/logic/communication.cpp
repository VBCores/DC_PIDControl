#include <cstdio>

#include "stm32g4xx_ll_utils.h"

#include <cyphal/allocators/o1/o1_allocator.h>
#include <cyphal/providers/G4CAN.h>
#include <voltbro/utils.hpp>
#include <voltbro/config/config.hpp>

#include "logic.h"

static CanardNodeID NODE_ID;
static uint8_t CYPHAL_HEALTH_STATUS = uavcan_node_Health_1_0_NOMINAL;
static uint8_t CYPHAL_MODE = uavcan_node_Mode_1_0_INITIALIZATION;

static bool _is_cyphal_on = false;
static std::shared_ptr<CyphalInterface> cyphal_interface;

UtilityConfig utilities(micros_64, error_handler);
static FDCAN_FilterTypeDef sFilterConfig;

ReservedObject<EchoReader> echo_reader;
ReservedObject<NodeInfoReader> node_info_reader;
ReservedObject<RegisterAccessReader> register_access_reader;
ReservedObject<RegisterListReader> register_list_reader;

#define DIP_PIN_INFO(ind)                                         \
    PinInfo {                                                     \
        .pin_num = DIP_SW##ind##_Pin, .port = DIP_SW##ind##_GPIO_Port \
    }

ConfigFromPins<8> config_pins({
      DIP_PIN_INFO(1),
      DIP_PIN_INFO(2),
      DIP_PIN_INFO(3),
      DIP_PIN_INFO(4),
      DIP_PIN_INFO(5),
      DIP_PIN_INFO(6),
      DIP_PIN_INFO(7),
      DIP_PIN_INFO(8)
});

#define DIAGNOSTIC_ON
#define SELF_TEST_ON

#define MOTOR_ON_REG_NAME_LEN 11
#define MOTOR_SPEED_REG_NAME_LEN 11
#define MOTOR_CURRENT_LIM_REG_NAME_LEN 19

bool was_initialized = false;

uint8_t motor_on_reg_name[MOTOR_ON_REG_NAME_LEN + 1] = "motor.is_on";
uint8_t motor_current_lim_reg_name[MOTOR_CURRENT_LIM_REG_NAME_LEN + 1] = "motor.current_limit";
uint8_t motor_speed_reg_name[MOTOR_SPEED_REG_NAME_LEN + 1] = "motor.speed";

void run_self_diagnostic() {
#ifdef SELF_TEST_ON
    send_diagnostic((char*)"Starting self diagnostic\0");
    // TODO: test different components
    send_diagnostic((char*)"Self diagnostic complete\0");
#endif
}

void send_diagnostic(char* string) {
#ifdef DIAGNOSTIC_ON
    static uint8_t sd_buf[UString::buffer_size];
    static CanardTransferID sd_transfer_id = 0;

    UString::Type sd = {};

    sprintf((char*)sd.value.elements, "NODE ID: %d; MESSAGE: <%s>", get_node_id(), string);
    sd.value.count = strlen((char*)sd.value.elements);

    get_interface()->send_msg<UString>(
            &sd,
            sd_buf,
            SELF_DIAG_PORT,
            &sd_transfer_id
    );
#endif
}

void EchoReader::handler(
    const voltbro_echo_echo_service_Request_1_0& request,
    CanardRxTransfer* transfer
) {
    static uint8_t echo_buf[EchoResponse::buffer_size];
    EchoResponse::Type echo_response = {.pong = request.ping};
    get_interface()->send_response<EchoResponse>(
        &echo_response,
        echo_buf,
        transfer,
        ECHO_SERVICE_ID
    );
}

void NodeInfoReader::handler(
    const uavcan_node_GetInfo_Request_1_0& object,
    CanardRxTransfer* transfer
) {
    static uint8_t node_info_buf[NodeInfoResponse::buffer_size];

    NodeInfoResponse::Type node_info_response = {
        .protocol_version =
            {
                1,
                0,
            },
        .hardware_version =
            {
                    1,
                    0,
            },
        .software_version = {0, 1},
        .software_vcs_revision_id = VCS_REVISION_ID};
    node_info_response.certificate_of_authenticity.count = 0;
    node_info_response.software_image_crc.count = 0;

    size_t name_len;
    switch (get_node_id()) {
        case 2:
            name_len = 22;
            memcpy(node_info_response.name.elements, "org.voltbro.motor.left", name_len);
            break;
        case 4:
            name_len = 23;
            memcpy(node_info_response.name.elements, "org.voltbro.motor.right", name_len);
            break;
        default:
            name_len = 7;
            memcpy(node_info_response.name.elements, "unknown", name_len);
            break;
    }
    node_info_response.name.count = name_len;

    uint32_t word0 = LL_GetUID_Word0();
    uint32_t word1 = LL_GetUID_Word1();
    uint32_t word2 = LL_GetUID_Word2();
    memcpy(node_info_response.unique_id, &word0, 4);
    memcpy(node_info_response.unique_id + 4, &word1, 4);
    memcpy(node_info_response.unique_id + 8, &word2, 4);

    node_info_response.unique_id[0] = get_node_id();

    get_interface()->send_response<NodeInfoResponse>(
        &node_info_response,
        node_info_buf,
        transfer,
        uavcan_node_GetInfo_1_0_FIXED_PORT_ID_
    );
}

void RegisterAccessReader::handler(
        const uavcan_register_Access_Request_1_0& register_access_request,
        CanardRxTransfer* transfer
) {
    static uint8_t register_access_response_buf[RegisterAccessResponse::buffer_size];
    RegisterAccessResponse::Type register_access_response = {};

    register_access_response.timestamp.microsecond = micros_64();
    uavcan_register_Value_1_0 value = {};
    DCMotorController& motor = get_motor();
    if (memcmp(
            register_access_request.name.name.elements,
            motor_on_reg_name,
            MOTOR_ON_REG_NAME_LEN
    ) == 0) {
        if (register_access_request.value._tag_ == 11) {
            if (register_access_request.value.natural8.value.elements[0] > 0) {
                motor.start();
                if (!was_initialized) {
                    was_initialized = true;
                }
            } else {
                motor.stop();
                motor.set_target_speed(0);
            }
        }

        register_access_response.persistent = true;
        register_access_response._mutable = true;
        value._tag_ = 11;
        uavcan_primitive_array_Natural8_1_0 result = {};
        result.value.elements[0] = (uint8_t)motor.is_on();
        result.value.count = 1;
        value.natural8 = result;
    } else if (memcmp(register_access_request.name.name.elements, motor_speed_reg_name, MOTOR_SPEED_REG_NAME_LEN) == 0) {
        if (register_access_request.value._tag_ == 12) {
            double new_speed = register_access_request.value.real64.value.elements[0];
            motor.set_target_speed(new_speed);
        }

        register_access_response.persistent = true;
        register_access_response._mutable = true;
        value._tag_ = 12;
        uavcan_primitive_array_Real64_1_0 result = {};
        result.value.elements[0] = motor.get_speed();
        result.value.count = 1;
        value.real64 = result;
    } else if (memcmp(register_access_request.name.name.elements, motor_current_lim_reg_name, MOTOR_CURRENT_LIM_REG_NAME_LEN) == 0) {
        if (register_access_request.value._tag_ == 12) {
            double new_current_lim = register_access_request.value.real64.value.elements[0];
            if (new_current_lim > 0) {
                motor.set_Ipeak(new_current_lim);
            }
        }

        register_access_response.persistent = true;
        register_access_response._mutable = true;
        value._tag_ = 12;
        uavcan_primitive_array_Real64_1_0 result = {};
        result.value.elements[0] = motor.get_Ipeak();
        result.value.count = 1;
        value.real64 = result;
    } else {
        value._tag_ = 0;
        value.empty = (uavcan_primitive_Empty_1_0){};
    }
    register_access_response.value = value;

    get_interface()->send_response<RegisterAccessResponse>(
        &register_access_response,
        register_access_response_buf,
        transfer,
        uavcan_register_Access_1_0_FIXED_PORT_ID_
    );
}

void RegisterListReader::handler(
    const uavcan_register_List_Request_1_0& register_list_request,
    CanardRxTransfer* transfer
) {
    static uint8_t register_list_response_buf[RegisterListResponse::buffer_size];
    RegisterListResponse::Type register_list_response = {};

    uavcan_register_Name_1_0 name = {};
    uint8_t* reg_name = nullptr;
    size_t reg_name_len = 0;
    switch (register_list_request.index) {
        case 0:
            reg_name = motor_on_reg_name;
            reg_name_len = MOTOR_ON_REG_NAME_LEN;
            break;
        case 1:
            reg_name = motor_speed_reg_name;
            reg_name_len = MOTOR_SPEED_REG_NAME_LEN;
            break;
        case 2:
            reg_name = motor_current_lim_reg_name;
            reg_name_len = MOTOR_CURRENT_LIM_REG_NAME_LEN;
            break;
    }
    if (reg_name != nullptr) {
        memcpy(name.name.elements, reg_name, reg_name_len);
        name.name.count = reg_name_len;
    }
    register_list_response.name = name;

    get_interface()->send_response<RegisterListResponse>(
            &register_list_response,
            register_list_response_buf,
            transfer,
            uavcan_register_List_1_0_FIXED_PORT_ID_
    );
}

static uint32_t uptime = 0;
void heartbeat() {
    static uint8_t hbeat_buffer[HBeat::buffer_size];
    static CanardTransferID hbeat_transfer_id = 0;
    HBeat::Type heartbeat_msg = {
            .uptime = uptime,
            .health = {CYPHAL_HEALTH_STATUS},
            .mode = {CYPHAL_MODE}
    };
    get_interface()->send_msg<HBeat>(
            &heartbeat_msg,
            hbeat_buffer,
            uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
            &hbeat_transfer_id,
            MICROS_S * 2
    );
}

static millis report_time = 0;
static millis heartbeat_time = 0;
void reporting_loop(
    millis millis,
    DCMotorController& motor
) {
    if (_is_cyphal_on) {
        EACH_N(millis, report_time, 100, {
            send_angle(motor.get_angle());
            send_angular_vel(motor.get_speed());
        })
        EACH_N(millis, heartbeat_time, 1000, {
            uptime++;
            heartbeat();
        })
    }
}

bool is_cyphal_on() {
    return _is_cyphal_on;
}

void setup_cyphal() {
    NODE_ID = config_pins.get_id();

    cyphal_interface = std::shared_ptr<CyphalInterface>(CyphalInterface::create_heap<G4CAN, O1Allocator>(
        NODE_ID,
        &hfdcan1,
        200,
        utilities
    ));

    echo_reader.create(cyphal_interface);
    node_info_reader.create(cyphal_interface);
    register_list_reader.create(cyphal_interface);
    register_access_reader.create(cyphal_interface);

    CanardFilter cyphal_filter = canardMakeFilterForServices(NODE_ID);

    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = cyphal_filter.extended_can_id;
    sFilterConfig.FilterID2 = cyphal_filter.extended_mask;

    HAL_IMPORTANT(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig))

    /* FROM STM EXAMPLES:
     * "Configure and enable Tx Delay Compensation, required for BRS mode.
     * TdcOffset default recommended value: DataTimeSeg1 * DataPrescaler
     * TdcFilter default recommended value: 0"
     */
    HAL_IMPORTANT(HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 14, 0))
    HAL_IMPORTANT(HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1))

    HAL_IMPORTANT(HAL_FDCAN_Start(&hfdcan1))

    _is_cyphal_on = true;
}

void send_angular_vel(float angular_velocity) {
    const CanardPortID AV_PORT = 6586;  // ascii, 65 - A, 86 - V, 6586 - AV: 'A'ngular 'V'elocity
    static uint8_t angular_vel_buffer[AngularVelocityScalar::buffer_size];
    static CanardTransferID angular_vel_transfer_id = 0;

    AngularVelocityScalar::Type angular_vel_msg = {.radian_per_second = angular_velocity};
    get_interface()->send_msg<AngularVelocityScalar>(
            &angular_vel_msg,
            angular_vel_buffer,
            AV_PORT,
            &angular_vel_transfer_id,
            MICROS_S / 10
    );
}

void send_angle(float angle) {
    const CanardPortID ANG_PORT = 6578;  // ascii, 65 - A, 78 - N, 6578 - AN: Angle
    static uint8_t angle_buffer[AngleScalar::buffer_size];
    static CanardTransferID angle_transfer_id = 0;

    AngleScalar::Type angle_msg = {.radian = angle};
    get_interface()->send_msg<AngleScalar>(
            &angle_msg,
            angle_buffer,
            ANG_PORT,
            &angle_transfer_id,
            MICROS_S / 10
    );
}

CanardNodeID get_node_id() {
    return NODE_ID;
}

inline bool isInterrupt()
{
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0 ;
}

std::shared_ptr<CyphalInterface> get_interface() {
#ifdef DEBUG
    /* Disallow calls from interrupts to avoid memory and/or queue corruption
     * If you need to call cyphal from interrupts, either:
     *     a) Wrap all calls to cyphal_interface into CRITICAL_SECTION(...) (this way has it's own problems)
     *     b) Implement something like a mutex (ex. https://gist.github.com/hdznrrd/4032002)
     */
    if (isInterrupt()) {
        error_handler();
    }
#endif
    return cyphal_interface;
}

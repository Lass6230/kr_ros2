/*////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2022 by Kassow Robots, ApS
//
// The information contained herein is confidential, proprietary to Kassow Robots,
// ApS, and considered a trade secret as defined in section 263 and section 264
// under the Danish Criminal Code. Use of this information by anyone
// other than authorized employees of Kassow Robots, ApS is granted only under a
// written non-disclosure agreement, expressly prescribing the scope and
// manner of such use.
//
// Authors: Ondrej Bruna  obr@kassowrobots.com
//
/////////////////////////////////////////////////////////////////////////////*/

#ifndef KR2_KORD_API_REQUEST_H
#define KR2_KORD_API_REQUEST_H

#pragma once

#include "kr2/kord/protocol/DataDescriptions/Requests/ControlCommandItems.h"
#include "kr2/kord/protocol/DataDescriptions/Requests/ControlCommandStatus.h"
#include "kr2/kord/protocol/DataDescriptions/Requests/RCAPICommandItems.h"
#include "kr2/kord/protocol/DataDescriptions/Requests/ServerServiceCommands.h"
#include "kr2/kord/protocol/KORDItemIDs.h"
#include "kr2/kord/protocol/ServerParameters.h"
#include "kr2/kord/protocol/ServerServiceIDs.h"

#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

namespace kr2::kord {

namespace kkp = kr2::kord::protocol;

class Request {
public:
    // This is a unique identifier to the request (based on a nanosec timestamp)
    int64_t request_rid_{0};
    kkp::EKORDItemID request_type_{};
    kkp::EControlCommandItems system_request_type_{};
    kkp::EControlCommandStatus request_status_{};
    unsigned int error_code_{};

    virtual ~Request() = default;

    Request &asSystem()
    {
        this->request_type_ = kkp::EKORDItemID::eRequestSystem;

        return *this;
    }
    // RequestIO& asIO();

    [[nodiscard]] kkp::EControlCommandStatus getStatus() const { return request_status_; }
};

class RequestSystem : public Request {

public:
    RequestSystem() { request_type_ = kkp::EKORDItemID::eRequestSystem; };
    explicit RequestSystem(const Request &a_req) : Request(a_req){};

    /**
     * @brief Set command item to \b TransferLogFiles command and fills in the
     * request identifier.
     *
     * @return RequestSystem&
     */
    RequestSystem &asLogTransfer()
    {
        system_request_type_ = kkp::EControlCommandItems::eTransferLogFiles;
        request_rid_ = std::chrono::steady_clock::now().time_since_epoch().count();
        return *this;
    };

    /**
     * @brief Set command item to \b TransferDashboardJson command and fills in the
     * request identifier.
     *
     * @return RequestSystem&
     */
    RequestSystem &asDashboardJsonTransfer()
    {
        system_request_type_ = kkp::EControlCommandItems::eTransferDashboardJson;
        request_rid_ = std::chrono::steady_clock::now().time_since_epoch().count();
        return *this;
    };

    /**
     * @brief Set command item to \b TransferCalibrationData command and fills in the
     * request identifier.
     *
     * @return RequestSystem&
     */
    RequestSystem &asCalibrationDataTransfer()
    {
        system_request_type_ = kkp::EControlCommandItems::eTransferCalibrationData;
        request_rid_ = std::chrono::steady_clock::now().time_since_epoch().count();
        return *this;
    };

    RequestSystem &asServerCommunication(bool enable)
    /**
     * @brief Enable/disable server communication capabilities of KORD.
     * Note: Server communication should be DISABLED when running in real-time mode!
     *
     * @param enable Enable server communication if true, false otherwise.
     * @return RequestSystem&
     */
    {
        if (enable) {
            system_request_type_ = kkp::EControlCommandItems::eServerEnableCommunication;
        }
        else {
            system_request_type_ = kkp::EControlCommandItems::eServerDisableCommunication;
        }
        return *this;
    }
};

class RequestTransfer : public Request {

public:
    uint32_t tranfer_mask_{0};

    RequestTransfer() { request_type_ = kkp::EKORDItemID::eRequestTransfer; };
    explicit RequestTransfer(const Request &a_req) : Request(a_req){};

    /**
     * @brief Set command item to \b TransferFiles command and fills in the
     * request identifier and files mask.
     *
     * @return RequestTransfer&
     */
    RequestTransfer &asMultipleFilesTransfer(uint32_t a_mask)
    {
        system_request_type_ = kkp::EControlCommandItems::eTransferFiles;
        request_rid_ = std::chrono::steady_clock::now().time_since_epoch().count();
        tranfer_mask_ = a_mask;
        return *this;
    };

    /**
     * @brief Set command item to \b TransferFiles command and fills in the
     * request identifier and files mask.
     *
     * @return RequestTransfer&
     */
    RequestTransfer &asFilesTransfer()
    {
        system_request_type_ = kkp::EControlCommandItems::eTransferFiles;
        request_rid_ = std::chrono::steady_clock::now().time_since_epoch().count();
        tranfer_mask_ = 0;
        return *this;
    };

    /**
     * @brief Add Logs to the \b TransferFiles files
     *
     *
     * @return RequestTransfer&
     */
    RequestTransfer &withLogs()
    {
        tranfer_mask_ |= 1 << 0;
        return *this;
    };

    /**
     * @brief Add DashboardJson to the \b TransferFiles files
     *
     * @return RequestTransfer&
     */
    RequestTransfer &withDashboardJson()
    {
        tranfer_mask_ |= 1 << 1;
        return *this;
    };

    /**
     * @brief Add Calibration file, model.urdf, to the \b TransferFiles files
     *
     * @return RequestTransfer&
     */
    RequestTransfer &withCalibration()
    {
        tranfer_mask_ |= 1 << 2;
        return *this;
    };
};

class RequestRCAPICommand : public Request {

public:
    uint16_t command_id_{0};
    uint32_t payload_length_{0};
    std::vector<uint8_t> payload_{{}};

    RequestRCAPICommand() { request_type_ = kkp::EKORDItemID::eCommandRCAPI; };
    explicit RequestRCAPICommand(const Request &a_req) : Request(a_req){};

    /**
     * @brief Set command item to user consent
     *
     * @return RequestRCAPICommand&
     */
    RequestRCAPICommand &asUserConsent()
    {
        system_request_type_ = kkp::EControlCommandItems::eRCAPICommand;
        command_id_ = kkp::ERCAPICommandIds::eUserConsent;
        payload_ = {};
        return *this;
    };

    RequestRCAPICommand &addPayload(uint8_t byte)
    {
        payload_length_ += 1;
        payload_.push_back(byte);
        return *this;
    };
};

class RequestServer : public Request {
public:
    RequestServer() { request_type_ = kkp::EKORDItemID::eRequestServer; }
    explicit RequestServer(const Request &a_req) : Request(a_req){};

    /**
     * @param a_command Action that should be applied to the service.
     * @param a_service_id Service ID represents the service that should be command
     * applied to.
     * @return RequestServerCommand&
     */
    RequestServer &asServiceRequest(kkp::EServerServiceCommands a_command, kkp::EKORDServerServiceID a_service_id)
    {
        system_request_type_ = kkp::EControlCommandItems::eServer;
        command_ = static_cast<uint16_t>(a_command);
        service_id_ = static_cast<uint16_t>(a_service_id);
        parameters_ = nullptr;
        return *this;
    }

    /**
     * @param a_command Action that should be applied to the service.
     * @param a_service_id Service ID represents the service that should be command
     * applied to.
     * @param a_payload Payload holds parameters that should be used in a request.
     * @return RequestServerCommand&
     */
    RequestServer &asServiceRequest(kkp::EServerServiceCommands a_command,
                                    kkp::EKORDServerServiceID a_service_id,
                                    std::shared_ptr<kkp::ServiceParameters> a_parameters)
    {
        system_request_type_ = kkp::EControlCommandItems::eServer;
        command_ = static_cast<uint16_t>(a_command);
        service_id_ = static_cast<uint16_t>(a_service_id);
        parameters_ = a_parameters;
        return *this;
    }

    uint16_t command_{};
    uint16_t service_id_{};
    std::shared_ptr<kkp::ServiceParameters> parameters_;
};

} // namespace kr2::kord

#endif // KR2_KORD_API_REQUEST_H
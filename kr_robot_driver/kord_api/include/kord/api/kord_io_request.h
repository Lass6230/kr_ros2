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
// Authors: Alexander Kazakov aka@kassowrobots.com
//
/////////////////////////////////////////////////////////////////////////////*/

#ifndef KR2_KORD_IO_REQUEST_H
#define KR2_KORD_IO_REQUEST_H

#pragma once

#include "kord/api/api_request.h"
#include "kr2/kord/io/IO.h"
#include "kr2/kord/protocol/KORDItemIDs.h"

namespace kr2::kord {

namespace kkp = kr2::kord::protocol;

class RequestIO : public Request {

public:
    enum DIGITAL_RELAYS { RELAY1 = MASK_RELAY1, RELAY2 = MASK_RELAY2, RELAY3 = MASK_RELAY3, RELAY4 = MASK_RELAY4 };
    enum DIGITAL_IOBOARD {
        DO1 = MASK_DO1,
        DO2 = MASK_DO2,
        DO3 = MASK_DO3,
        DO4 = MASK_DO4,
        DO5 = MASK_DO5,
        DO6 = MASK_DO6,
        DO7 = MASK_DO7,
        DO8 = MASK_DO8
    };
    enum DIGITAL_IOTOOLB { TB1 = MASK_TB1, TB2 = MASK_TB2, TB3 = MASK_TB3, TB4 = MASK_TB4 };
    enum DIGITAL_SAFE { SDO1 = MASK_SDO1, SDO2 = MASK_SDO2, SDO3 = MASK_SDO3, SDO4 = MASK_SDO4 };

    int64_t mask_{};
    uint8_t value_{};
    int32_t config_id_{};

    RequestIO();
    explicit RequestIO(const Request &a_req);

    RequestIO &asSetIODigitalOut();

    RequestIO &asSetIOAnalogOut();

    RequestIO &withEnabledPorts(int64_t mask);

    RequestIO &withDisabledPorts(int64_t mask);

    RequestIO &withEnabledSafePorts(int64_t mask, ESafePortConfiguration config);
};

} // namespace kr2::kord

#endif
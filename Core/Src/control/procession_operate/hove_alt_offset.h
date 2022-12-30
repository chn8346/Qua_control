//
// Created by fanchunhui on 2022/12/30.
//

#ifndef QUA_CONTROL_HOVE_ALT_OFFSET_H
#define QUA_CONTROL_HOVE_ALT_OFFSET_H

#include "../../base_head.h"

// 标记飞行器的状态
#define HAO_STATE_LAND     0        // 落地
#define HAO_STATE_TAKEOFF  1        // 起飞
#define HAO_STATE_LANDING  2        // 正在落地
#define HAO_STATE_AIR      4        // 空中


class hove_alt_offset {
public:
    uint8_t hao_state = HAO_STATE_LAND; // 默认已经落地

    void set_hao_state(uint8_t state);

private:

};


#endif //QUA_CONTROL_HOVE_ALT_OFFSET_H

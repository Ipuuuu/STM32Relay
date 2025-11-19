#pragma once

namespace Relay {
    struct Pin {
        int number;
        constexpr Pin(int num) : number(num) {}
    };
}

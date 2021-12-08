#include "I_Print.hpp"

std::ostream& operator<<(std::ostream& out, const I_Print& obj) {
    obj.print(out);
    return out;
}

I_Print::~I_Print() = default;
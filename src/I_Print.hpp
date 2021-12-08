#pragma once
#include <iostream>

class I_Print {
    friend std::ostream& operator<<(std::ostream&, const I_Print&);
public:
    virtual void print(std::ostream&) const = 0;
    virtual ~I_Print();
};
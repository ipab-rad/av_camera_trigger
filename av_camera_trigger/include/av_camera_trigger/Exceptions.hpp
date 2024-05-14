#pragma once

#include <stdexcept>

namespace except
{
    /// Thrown when the NUCLEO board is disconnected and an attempt
    /// to execute a query is made.
    struct BrokenChannel : public std::runtime_error
    {
        using std::runtime_error::runtime_error;
    };

    /// Thrown when the NUCLEO board is @em connected but the execution of
    /// a query fails due to other factors which have to do with the overall
    /// logic.
    struct Logic : public std::logic_error
    {
        using std::logic_error::logic_error;
    };
}

#pragma once

#include <cmath>

#include "lowpassfilter2p.h"

namespace PayloadMPC
{

struct VerticalDisturbanceLowPassResult
{
    double value{0.0};
    bool valid{false};
};

class VerticalDisturbanceLowPass
{
public:
    bool configure(double sample_frequency, double cutoff_frequency)
    {
        configured_ = std::isfinite(sample_frequency) && sample_frequency > 0.0 &&
            std::isfinite(cutoff_frequency) && cutoff_frequency >= 0.0 &&
            cutoff_frequency < sample_frequency / 2.0;
        enabled_ = configured_ && cutoff_frequency > 0.0;
        if (!configured_)
        {
            filter_.disable();
            return false;
        }

        if (enabled_)
            filter_.set_cutoff_frequency(sample_frequency, cutoff_frequency);
        else
            filter_.disable();
        reset();
        return true;
    }

    VerticalDisturbanceLowPassResult update(double input)
    {
        VerticalDisturbanceLowPassResult result;
        if (!configured_ || !std::isfinite(input))
        {
            reset();
            return result;
        }

        result.value = enabled_ ? filter_.apply(input) : input;
        result.valid = std::isfinite(result.value);
        if (!result.valid)
            reset();
        return result;
    }

    void reset()
    {
        if (enabled_)
            filter_.reset(0.0);
    }

    bool enabled() const
    {
        return enabled_;
    }

private:
    LowPassFilter2p<double> filter_;
    bool configured_{false};
    bool enabled_{false};
};

} // namespace PayloadMPC

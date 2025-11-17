#include <algorithm>
#include <cstdlib>

namespace Math
{
    inline float MoveTowards(float from, float to, float amount)
    {
        float diff = to - from;
        if (diff == 0)
            return from;

        float sign = diff / std::abs(diff);
        float delta = std::min(amount, std::abs(diff));
        
        return from + sign * delta;
    }
} // namespace Math
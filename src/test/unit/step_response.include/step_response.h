#include <functional>

namespace step_response {

using ResponseFn = std::function<float(void*)>;
class StepResponse {
  public:
    StepResponse(ResponseFn fn, int iterations, void *param,
                 float error_band)
        : error_band_(error_band), iterations_(iterations)
    {
        data_ = std::make_unique<float[]>(iterations);
        for (int i = 0; i < iterations; i++) {
            data_[i] = fn(param);
        }

        const float settled_value = data_[iterations - 1];

        // Settle time
        const float low = settled_value - error_band_;
        const float high = settled_value + error_band_;
        for (int i = iterations - 1; i != 0; i--) {
            if (data_[i] < low || data_[i] > high) {
                settle_time_ = i + 1;
                break;
            }
        }

        // Overshoot
        overshoot_ = 0;
        overshoot_time_ = -1;
        for (int i = 0; i < settle_time_; i++) {
            float diff = data_[i] - settled_value;
            if (diff > overshoot_) {
                overshoot_ = diff;
                overshoot_time_ = i;
            }
        }
    }
    // Returns the iteration number
    int GetSettleTime() {
        return settle_time_;
    }

    float GetOvershoot() {
        return overshoot_;
    }

    int GetOvershootTime() {
        return overshoot_time_;
    }

    int iterations_;
    std::unique_ptr<float[]> data_;
    float error_band_;
    int settle_time_ = 0;
    int overshoot_time_ = 0;
    float overshoot_ = 0;
};

}

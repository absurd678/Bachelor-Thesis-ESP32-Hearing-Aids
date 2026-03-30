#pragma once

class LMSFilter {
public:
    LMSFilter(int taps, float mu)
        : taps(taps), mu(mu), w(new float[taps]()), x(new float[taps]()) {}

    ~LMSFilter() {
        delete[] w;
        delete[] x;
    }

    LMSFilter(const LMSFilter&) = delete;
    LMSFilter& operator=(const LMSFilter&) = delete;

    float process(float reference, float desired) {
        for (int i = taps - 1; i > 0; --i) {
            x[i] = x[i - 1];
        }
        x[0] = reference;

        float estimate = 0.0f;
        float power = 1e-9f;
        for (int i = 0; i < taps; ++i) {
            estimate += w[i] * x[i];
            power += x[i] * x[i];
        }

        const float error = desired - estimate;
        const float step = mu / power;
        for (int i = 0; i < taps; ++i) {
            w[i] += step * error * x[i];
        }

        return error;
    }

private:
    int taps;
    float mu;
    float* w;
    float* x;
};

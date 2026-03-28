class LMSFilter {
public:
    LMSFilter(int taps, float mu)
        : taps(taps), mu(mu) {
        w = new float[taps]();
        x = new float[taps]();
    }

    ~LMSFilter() {
        delete[] w;
        delete[] x;
    }

    float process(float input, float desired) {
        // сдвиг буфера
        for (int i = taps - 1; i > 0; i--) {
            x[i] = x[i - 1];
        }
        x[0] = input;

        // выход фильтра
        float y = 0;
        for (int i = 0; i < taps; i++) {
            y += w[i] * x[i];
        }

        // ошибка
        float e = desired - y;

        // обновление коэффициентов LMS
        for (int i = 0; i < taps; i++) {
            w[i] += 2 * mu * e * x[i];
        }

        return y;
    }

private:
    int taps;
    float mu;
    float* w;
    float* x;
};
#include "odometry.h"
#include <assert.h>
#include <math.h>

int main(void)
{
    OdomEstimator_t estimator;
    OdomEstimator_Init(&estimator, NULL);

    estimator.last_yaw_rad = 1.25f;
    estimator.heading_rad = -0.75f;
    estimator.yaw_ready = 1U;
    for (uint8_t i = 0U; i < 4U; i++)
    {
        estimator.last_pos_deg[i] = 100.0f + (float)i;
        estimator.origin_pos_deg[i] = 50.0f + (float)i;
        estimator.wheel_ready[i] = 1U;
        estimator.origin_ready[i] = 1U;
    }

    OdomEstimator_ResetDistanceOrigin(&estimator);

    assert(fabsf(estimator.last_yaw_rad - 1.25f) < 0.0001f);
    assert(fabsf(estimator.heading_rad + 0.75f) < 0.0001f);
    assert(estimator.yaw_ready == 1U);
    assert(fabsf(estimator.config.wheel_radius_m - 0.025f) < 0.0001f);
    for (uint8_t i = 0U; i < 4U; i++)
    {
        assert(estimator.last_pos_deg[i] == 0.0f);
        assert(estimator.origin_pos_deg[i] == 0.0f);
        assert(estimator.wheel_ready[i] == 0U);
        assert(estimator.origin_ready[i] == 0U);
    }

    return 0;
}

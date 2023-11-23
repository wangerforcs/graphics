#include "solver.h"

#include <Eigen/Core>

using Eigen::Vector3f;

// External Force does not changed.

// Function to calculate the derivative of KineticState
KineticState derivative(const KineticState& state)
{
    return KineticState(state.velocity, state.acceleration, Eigen::Vector3f(0, 0, 0));
}

// Function to perform a single Forward Euler step
KineticState forward_euler_step([[maybe_unused]] const KineticState& previous,
                                const KineticState& current)
{
    KineticState next;

    next.position = current.position + current.velocity * time_step;
    next.velocity = current.velocity + current.acceleration * time_step;
    next.acceleration = current.acceleration;
    return next;
}

// Function to perform a single Runge-Kutta step
KineticState runge_kutta_step([[maybe_unused]] const KineticState& previous,
                              const KineticState& current)
{
    KineticState next;
    next.position = current.position + current.velocity * time_step + 0.5f * current.acceleration * time_step * time_step;
    next.velocity = current.velocity + current.acceleration * time_step;
    next.acceleration = current.acceleration;
    return current;
}

// Function to perform a single Backward Euler step
KineticState backward_euler_step([[maybe_unused]] const KineticState& previous,
                                 const KineticState& current)
{
    KineticState next;
    next.velocity = current.velocity + current.acceleration * time_step;
    next.position = current.position + next.velocity * time_step;
    next.acceleration = current.acceleration;
    return next;
}

// Function to perform a single Symplectic Euler step
KineticState symplectic_euler_step(const KineticState& previous, const KineticState& current)
{
    (void)previous;
    KineticState next;
    next.velocity = current.velocity + current.acceleration * time_step;
    next.position = current.position + next.velocity * time_step;
    next.acceleration = current.acceleration;
    return next;
}

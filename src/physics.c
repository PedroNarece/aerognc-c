#include "physics.h"

// Constante padrão de gravidade padrão para cálculo do Isp (foguetes da NASA/ESA)
#define STANDARD_GRAVITY 9.80665

double physics_get_total_mass(const Spacecraft *ship) {
    return ship->dry_mass + ship->fuel_mass;
}

void physics_update(Spacecraft *ship, const Environment *env, double dt) {
    // 1. Defesa: Limitar o acelerador rigorosamente entre 0.0 e 1.0 (0% a 100%)
    if (ship->throttle < 0.0) {
        ship->throttle = 0.0;
    }
    if (ship->throttle > 1.0) {
        ship->throttle = 1.0;
    }

    // 2. Calcula o empuxo pretendido
    double actual_thrust = ship->max_thrust * ship->throttle;

    // 3. Verificação de combustível: se o tanque estiver seco, empuxo é zero!
    if (ship->fuel_mass <= 0.0) {
        ship->fuel_mass = 0.0;
        actual_thrust = 0.0;
    } else {
        // Se o motor está ativo, consumimos combustível pela equação de foguetes:
        // Consumo por segundo (kg/s) = Empuxo / (Isp * g0)
        double mass_flow_rate = actual_thrust / (ship->isp * STANDARD_GRAVITY);
        double fuel_consumed = mass_flow_rate * dt;

        // Deduzimos o combustível consumido
        ship->fuel_mass -= fuel_consumed;

        // Se o combustível acabou no meio desse ciclo
        if (ship->fuel_mass < 0.0) {
            ship->fuel_mass = 0.0;
        }
    }

    // 4. Massa total atual da nave (massa seca + o que restou de combustível)
    double total_mass = physics_get_total_mass(ship);

    // 5. Segunda Lei de Newton: a = (T / m) - g
    double acceleration = (actual_thrust / total_mass) - env->gravity;

    // 6. Integração Numérica de Euler
    ship->velocity += acceleration * dt;
    ship->altitude += ship->velocity * dt;

    // 7. Limite de solo: a nave não pode atravessar o chão sólido
    if (ship->altitude < 0.0) {
        ship->altitude = 0.0;
    }
}

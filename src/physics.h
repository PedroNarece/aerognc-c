#ifndef PHYSICS_H
#define PHYSICS_H

// Representa o estado físico da nave espacial
typedef struct {
    double altitude;       // Posição vertical em relação ao solo (m)
    double velocity;       // Velocidade vertical (m/s)
    double dry_mass;       // Massa da estrutura sem combustível (kg)
    double fuel_mass;      // Massa de combustível restante (kg)
    double max_thrust;     // Força máxima do motor (Newtons)
    double throttle;       // Acelerador: 0.0 (desligado) a 1.0 (potência máxima 100%)
    double isp;            // Impulso Específico (segundos) - mede a eficiência do motor
} Spacecraft;

// Representa os parâmetros do ambiente
typedef struct {
    double gravity;        // Aceleração da gravidade (m/s^2)
} Environment;

// Retorna a massa total atual da nave (massa seca + combustível)
double physics_get_total_mass(const Spacecraft *ship);

// Atualiza a física com motor ativo, gravidade e consumo de combustível
void physics_update(Spacecraft *ship, const Environment *env, double dt);

#endif // PHYSICS_H

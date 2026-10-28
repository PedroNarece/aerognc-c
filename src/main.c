#include <stdio.h>
#include "physics.h"
#include "gnc.h"

int main(void) {
    printf("=========================================================\n");
    printf("  AeroGNC-C: Pouso Autônomo com Autopiloto PID em Malha Fechada \n");
    printf("=========================================================\n\n");

    // 1. Configuração da Nave
    Spacecraft starship = {
        .altitude = 1000.0,      // Começa a 1000 metros
        .velocity = 0.0,         // Repouso inicial
        .dry_mass = 1000.0,      // 1000 kg de carcaça
        .fuel_mass = 600.0,      // 600 kg de combustível
        .max_thrust = 35000.0,   // 35 kN de empuxo máximo
        .throttle = 0.0,
        .isp = 310.0             // Eficiência do motor
    };

    Environment earth = {
        .gravity = 9.81
    };

    // 2. Inicializamos o Autopiloto PID (Ganhos ajustados)
    PIDController autopilot;
    pid_init(&autopilot, 0.08, 0.015, 0.04);

    double time = 0.0;
    double dt = 0.02; // Ciclo de 50 Hz (20 milissegundos por atualização)

    printf("Iniciando descida autônoma a partir de 1000 metros...\n\n");
    printf(" Tempo   | Altitude  | Vel Real   | Vel Alvo   | Motor | Combustivel\n");
    printf("---------|-----------|------------|------------|-------|------------\n");

    while (starship.altitude > 0.0) {
        // [GUIDANCE - ORIENTAÇÃO]: Calcula a velocidade segura para esta altitude
        double target_velocity = guidance_calculate_target_velocity(starship.altitude);

        // [CONTROL - PID]: Potência base para anular o peso da nave (Feedforward)
        double current_mass = physics_get_total_mass(&starship);
        double hover_throttle = (current_mass * earth.gravity) / starship.max_thrust;

        // O PID calcula o ajuste fino com base na diferença entre a meta e a realidade
        double throttle_adjustment = pid_compute(&autopilot, target_velocity, starship.velocity, dt);

        // O comando final do acelerador limitado entre 0% e 100%
        starship.throttle = hover_throttle + throttle_adjustment;
        if (starship.throttle < 0.0) starship.throttle = 0.0;
        if (starship.throttle > 1.0) starship.throttle = 1.0;

        // Imprime a telemetria a cada 1 segundo (a cada 50 ciclos)
        if ((int)(time * 50) % 50 == 0) {
            printf("%6.1f s | %8.1f m | %8.2f m/s | %8.2f m/s | %4.0f%% | %8.1f kg\n",
                   time,
                   starship.altitude,
                   starship.velocity,
                   target_velocity,
                   starship.throttle * 100.0,
                   starship.fuel_mass);
        }

        // Atualiza a simulação física
        physics_update(&starship, &earth, dt);

        time += dt;
    }

    // Diagnóstico do pouso
    printf("---------------------------------------------------------\n");
    printf("STATUS DE TOUCHDOWN (SOLO) AOS %.2f s\n", time);
    printf("Velocidade de contato: %.2f m/s (%.2f km/h)\n", starship.velocity, starship.velocity * 3.6);
    printf("Combustível restante: %.2f kg\n", starship.fuel_mass);

    if (starship.velocity >= -2.0 && starship.velocity <= 0.0) {
        printf("🏆 SUCESSO HISTÓRICO! Pouso suave perfeito! (Touchdown < 2.0 m/s)\n");
    } else if (starship.fuel_mass <= 0.0) {
        printf("💥 FALHA CRÍTICA: O combustível esgotou antes de tocar o solo!\n");
    } else {
        printf("💥 IMPACTO CATASTRÓFICO! Velocidade excessiva no toque.\n");
    }

    return 0;
}

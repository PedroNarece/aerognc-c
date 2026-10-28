#include "gnc.h"
#include <math.h>

void pid_init(PIDController *pid, double kp, double ki, double kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0;
    pid->previous_error = 0.0;
}

double pid_compute(PIDController *pid, double setpoint, double current_value, double dt) {
    // 1. Cálculo do erro
    double error = setpoint - current_value;

    // 2. Termo Proporcional (P)
    double p_term = pid->kp * error;

    // 3. Termo Integral (I) com anti-windup (limite para não explodir a memória)
    pid->integral += error * dt;
    if (pid->integral > 10.0) pid->integral = 10.0;
    if (pid->integral < -10.0) pid->integral = -10.0;
    double i_term = pid->ki * pid->integral;

    // 4. Termo Derivativo (D)
    double derivative = (error - pid->previous_error) / dt;
    double d_term = pid->kd * derivative;

    // 5. Atualiza o erro anterior para o próximo ciclo de relógio
    pid->previous_error = error;

    // 6. Saída consolidada
    return p_term + i_term + d_term;
}

double guidance_calculate_target_velocity(double altitude) {
    // Se estiver a menos de 5 metros do solo, velocidade suave de 1.0 m/s para o toque
    if (altitude <= 5.0) {
        return -1.0;
    }

    // Perfil suave de desaceleração constante (a = 2.0 m/s^2)
    // v = -sqrt(2 * a * h)
    double target = -sqrt(2.0 * 2.0 * altitude);

    // Limitador de segurança: nunca descer a mais de 45 m/s
    if (target < -45.0) {
        target = -45.0;
    }

    return target;
}

#ifndef GNC_H
#define GNC_H

// Estrutura que mantém o estado e a memória do controlador PID
typedef struct {
    double kp;              // Ganho Proporcional
    double ki;              // Ganho Integral
    double kd;              // Ganho Derivativo
    double integral;        // Acumulador de erro ao longo do tempo (I)
    double previous_error;  // Erro do ciclo anterior (usado para calcular a derivada D)
} PIDController;

// Inicializa o controlador PID zerando os acumuladores
void pid_init(PIDController *pid, double kp, double ki, double kd);

// Calcula a saída de controle com base no erro atual (Meta - Realidade)
double pid_compute(PIDController *pid, double setpoint, double current_value, double dt);

// Função de Guiamento: calcula a velocidade-alvo ideal de descida para cada altitude
double guidance_calculate_target_velocity(double altitude);

#endif // GNC_H

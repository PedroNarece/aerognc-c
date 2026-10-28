# AeroGNC-C — Simulador de Computador de Voo e Pouso Autônomo em C

> 🚀 Computador de bordo simulado em malha fechada com Guiamento (Guidance), Navegação e Controle PID inspirado na NASA (Apollo Guidance Computer) e nos pousos verticais da SpaceX.

[![Linguagem C](https://img.shields.io/badge/Linguagem-C99-00599C?style=for-the-badge&logo=c)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Compilador GCC](https://img.shields.io/badge/Compilador-GCC-FFCF36?style=for-the-badge&logo=gnu)](https://gcc.gnu.org)
[![Automação Makefile](https://img.shields.io/badge/Build-Makefile-000000?style=for-the-badge&logo=gnu-bash)](https://www.gnu.org/software/make/)
[![Controle](https://img.shields.io/badge/Controle-PID%20em%20Malha%20Fechada-brightgreen?style=for-the-badge)]()

🎯 **Pouso Suave Alcançado:** Touchdown a **-0.48 m/s** (limite seguro: $< 2.0\text{ m/s}$)  
⏱️ **Frequência de Controle Hard Real-Time:** **50 Hz** ($\Delta t = 20\text{ ms}$)  
💾 **Arquitetura de Memória:** C puro com alocação estática, ponteiros diretos e zero caixas-pretas.

---

## 📌 O Problema

Pousar um veículo aeroespacial verticalmente (propulsive retro-landing) é um dos problemas mais difíceis da engenharia de controle:

- **Massa Variável:** A cada fração de segundo com o motor acionado, toneladas de combustível são expelidas. O peso do veículo muda continuamente, alterando a resposta dinâmica da nave.
- **Janela Estreita de Aceleração:** Se os motores forem ligados cedo demais, o veículo consome todo o propelente pairando no ar e despenca sem combustível. Se forem ligados tarde demais, o empuxo disponível não é suficiente para anular a inércia, resultando em impacto catastrófico.
- **Ineficácia de Regras Fixas:** Um algoritmo baseado em condicionais fixas (`if/else`) não se adapta a oscilações de altitude e velocidade. 

A solução exige um computador de bordo com capacidade de controle dinâmico em tempo real contínuo.

---

## 🛰️ A Solução

O **AeroGNC-C** é um simulador de voo modular escrito em C99 que implementa a tríade aeroespacial clássica (**GNC**):

1. **Navigation (Navegação):** Rastreia com precisão dupla (`double`) o vetor de estado do veículo (altitude, velocidade vertical e massa residual de combustível).
2. **Guidance (Guiamento):** Calcula em tempo real o perfil parabólico de velocidade segura para cada altitude até o solo:
   $$v_{alvo} = -\sqrt{2 \cdot a_{descida} \cdot h}$$
3. **Control (Controle PID + Feedforward):**
   - **Feedforward ($T_{hover}$):** Calcula a potência básica necessária para anular instantaneamente o peso da gravidade ($m(t) \cdot g$).
   - **PID em Malha Fechada:** Modula continuamente o acelerador (*Throttle* de 0% a 100%) eliminando o erro entre a velocidade real e a velocidade-alvo.
4. **Propulsão e Dinâmica:** Resolve a Segunda Lei de Newton ($a = \frac{T}{m} - g$) e o consumo mássico pela equação de empuxo de foguetes com impulso específico ($I_{sp}$).

---

## 🏛️ Arquitetura do Sistema

```
                      ┌────────────────────────────────────────┐
                      │              main.c                    │
                      │  Loop de Voo Discreto (Passo: 20 ms)   │
                      └────────────────────────────────────────┘
                                     │            │
            1. QUAL A VELOCIDADE IDEAL?           │ 3. INTEGRAÇÃO FÍSICA
            (Guidance & PID Controller)           │ (Newton & Euler)
                                     ▼            ▼
                      ┌──────────────────┐    ┌──────────────────┐
                      │      gnc.c       │    │    physics.c     │
                      │  Modula Throttle │    │  Consome fuel    │
                      │   (0.0 a 1.0)    │    │  e freia a nave  │
                      └──────────────────┘    └──────────────────┘
```

---

## 📂 Estrutura de Arquivos

| Arquivo | Responsabilidade |
| :--- | :--- |
| [`src/physics.h`](file:///Users/Pedro/Documents/Pensando%20em%20projeto%20em%20C%20/src/physics.h) | Definição das structs de estado contíguo (`Spacecraft`, `Environment`) e contratos de física. |
| [`src/physics.c`](file:///Users/Pedro/Documents/Pensando%20em%20projeto%20em%20C%20/src/physics.c) | Integração numérica de Euler, cálculo do consumo de combustível pelo $I_{sp}$ e aplicação da 2ª Lei de Newton. |
| [`src/gnc.h`](file:///Users/Pedro/Documents/Pensando%20em%20projeto%20em%20C%20/src/gnc.h) | Interface do controlador `PIDController` e perfil de aproximação de descida. |
| [`src/gnc.c`](file:///Users/Pedro/Documents/Pensando%20em%20projeto%20em%20C%20/src/gnc.c) | Algoritmo PID completo (P, I com anti-windup, D) e função matemática de orientação. |
| [`src/main.c`](file:///Users/Pedro/Documents/Pensando%20em%20projeto%20em%20C%20/src/main.c) | Orquestrador da simulação a 50 Hz e relatório de touchdown. |
| [`Makefile`](file:///Users/Pedro/Documents/Pensando%20em%20projeto%20em%20C%20/Makefile) | Script de compilação automatizada com flags rigorosas (`-Wall -Wextra -std=c99 -pedantic -lm`). |

---

## 🗺️ Roadmap de Desenvolvimento

- [x] **Fase 1 — Núcleo de Física e Queda Livre:** Modelagem das structs em C, passo de tempo discreto ($\Delta t$), integrador de Euler e automação com `Makefile`.
- [x] **Fase 2 — Sistema de Propulsão e Massa Variável:** Equações de empuxo, perda contínua de massa no tanque de propelente e travas defensivas de motor.
- [x] **Fase 3 — Autopiloto GNC e Controle PID:** Loop fechado a 50 Hz, feedforward de gravidade, perfil de guiamento e validação de pouso suave ($< 2.0\text{ m/s}$).
- [ ] **Fase 4 — Telemetria Visual no Terminal:** Painel dinâmico em tempo real usando ANSI Escape Codes com barras gráficas de altitude e queima de combustível.
- [ ] **Fase 5 — Estimação com Filtro de Kalman (Expansão):** Injeção de ruído estocástico em sensores de radar e reconstrução do sinal com filtro univariado.

---

## ⚡ Como Compilar e Executar

### Pré-requisitos
- Compilador C (`gcc` ou `clang`)
- Utilitário `make`

### 1. Compilar o Projeto
```bash
make
```

### 2. Executar a Simulação
```bash
make run
# ou diretamente:
./aerognc
```

### 3. Limpar Arquivos de Build
```bash
make clean
```

---

## 👨‍💻 Contexto e Propósito

Projeto desenvolvido por **Pedro Narece Silva Guimarães**, estudante no 2º período de **Engenharia da Computação**.

**Metodologia:** Desenvolvimento com foco absoluto no entendimento profundo dos fundamentos de computação de baixo nível (gerenciamento de memória, ponteiros, passagem por referência, integradores numéricos e teoria clássica de controle), sem geração de código em "caixa-preta".

---

## 📄 Licença

Distribuído sob a licença **MIT**. Consulte `LICENSE` para mais informações.

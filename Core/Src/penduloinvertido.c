
#include "penduloinvertido.h"

#include "math.h"


// Variáveis internas
static float integral = 0.0f;
static float erro_anterior = 0.0f;


// Funções privadas
static void Pendulo_AutoTeste(Pendulo_t *p);
static void Pendulo_SwingUp(Pendulo_t *p);
static void Pendulo_Controle(Pendulo_t *p);
static void Pendulo_AtualizaAngulo(Pendulo_t *p);
static void Pendulo_AtualizaVelocidade(Pendulo_t *p);


// Inicialização
void Pendulo_Inicializa(Pendulo_t *p)
{
    p->encoder = 0;

    p->angulo = 0.0f;
    p->velocidade = 0.0f;
    p->pulso_motor = 0;

    p->kp = 0.0f;
    p->ki = 0.0f;
    p->kd = 0.0f;

    p->estado = PENDULO_AUTOTESTE;
}


// Loop 1ms
void Pendulo_Atualiza_1ms(Pendulo_t *p)
{
    Pendulo_AtualizaAngulo(p);
    Pendulo_AtualizaVelocidade(p);

    switch(p->estado)
    {
        case PENDULO_AUTOTESTE:
            Pendulo_AutoTeste(p);
            break;

        case PENDULO_SWINGUP:
            Pendulo_SwingUp(p);
            break;

        case PENDULO_CONTROLE:
            Pendulo_Controle(p);
            break;

        case PENDULO_ERRO:
            p->pulso_motor = 0;
            break;
    }
}


// Seta encoder
void Pendulo_SetaEncoder(Pendulo_t *p, int32_t encoder)
{
    p->encoder = encoder;
}


// Seta PID
void Pendulo_SetaPID(Pendulo_t *p, float kp, float ki, float kd)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
}


// Inicia SWINGUP
void Pendulo_IniciaSwingUp(Pendulo_t *p)
{
    p->estado = PENDULO_SWINGUP;
}


//Parar o pêndulo
void Pendulo_Parar(Pendulo_t *p)
{
    p->pulso_motor = 0;

    p->estado = PENDULO_ERRO;
}

// Pega estado
PenduloEstado_t Pendulo_PegaEstado(Pendulo_t *p)
{
    return p->estado;
}


// Auto teste
static void Pendulo_AutoTeste(Pendulo_t *p)
{
    p->estado = PENDULO_AUTOTESTE;
}


// SWING-UP
static void Pendulo_SwingUp(Pendulo_t *p)
{
    /*
     * ALGORITMO SWING-UP
     */

    if(fabsf(p->angulo) < 5.0f)
    {
        p->estado = PENDULO_CONTROLE;
    }
}


// Controle PID
static void Pendulo_Controle(Pendulo_t *p)
{
    float setpoint = 0.0f;
    float erro;
    float derivada;

    erro = setpoint - p->angulo;
    integral += erro * 0.001f;
    derivada = (erro - erro_anterior) / 0.001f;

    p->pulso_motor =
            (p->kp * erro) +
            (p->ki * integral) +
            (p->kd * derivada);

    erro_anterior = erro;


    /*
     * Se cair muito,
     * volta para swing-up
     */

    if(fabsf(p->angulo) > 30.0f)
    {
        p->estado = PENDULO_SWINGUP;
    }
}


// Ângulo

static void Pendulo_AtualizaAngulo(Pendulo_t *p)
{
    /*
     * Converter encoder em ângulo
     */

    p->angulo = (float)p->encoder * 0.01f;
}


//Velocidade
static void Pendulo_AtualizaVelocidade(Pendulo_t *p)
{
    static float angulo_anterior = 0.0f;

    p->velocidade =
            (p->angulo - angulo_anterior) / 0.001f;

    angulo_anterior = p->angulo;
}

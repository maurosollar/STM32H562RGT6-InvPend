
#include "penduloinvertido.h"

#include "math.h"


// Variáveis internas
static float integral = 0.0f;
static float erro_anterior = 0.0f;


// Funções privadas
static void Pendulo_SelfTest(Pendulo_t *p);
static void Pendulo_SwingUp(Pendulo_t *p);
static void Pendulo_Controle(Pendulo_t *p);
static void Pendulo_AtualizaAngulo(Pendulo_t *p);
static void Pendulo_AtualizaVelocidade(Pendulo_t *p);
static void Motor_Velocidade(Pendulo_t *p, uint8_t direcao);


/**
 * @brief Inicialização, basicamente passa as referências e inicia variáveis da estrutura
 */
void Pendulo_Inicializa(Pendulo_t *p, TIM_HandleTypeDef *htim_motor_pwm, uint32_t canal_pwm,
                        GPIO_TypeDef *chave_dir_port, uint16_t chave_dir_pin,
                        GPIO_TypeDef *chave_esq_port, uint16_t chave_esq_pin,
	                	GPIO_TypeDef *direcao_port, short unsigned int direcao_pin)
{
    p->encoder = 0;
    p->angulo = 0.0f;
    p->velocidade = 0.0f;
    p->posicao_carro = 0.0f;
    p->pulso_motor = 0;

    p->kp = 0.0f;
    p->ki = 0.0f;
    p->kd = 0.0f;

    p->htim_motor_pwm = htim_motor_pwm;
    p->canal_pwm = canal_pwm;
    p->chave_dir_port = chave_dir_port;
    p->chave_dir_pin = chave_dir_pin;
    p->chave_esq_port = chave_esq_port;
    p->chave_esq_pin = chave_esq_pin;
    p->direcao_port = direcao_port;
    p->direcao_pin = direcao_pin;

    p->estado = PENDULO_SELFTEST;
}


/**
 * @brief Atualiza pêndulo dentro dos seus estados.
 */
void Pendulo_AtualizaPendulo(Pendulo_t *p)
{
    Pendulo_AtualizaAngulo(p);
    Pendulo_AtualizaVelocidade(p);

    switch(p->estado)
    {
        case PENDULO_SELFTEST:
            Pendulo_SelfTest(p);
        break;

        case PENDULO_SWINGUP:
            Pendulo_SwingUp(p);
            break;

        case PENDULO_CONTROLE:
            Pendulo_Controle(p);
            break;

        case PENDULO_ERRO:
            Pendulo_Parar(p);
            break;
    }
}

/**
 * @brief Altera velocidade do motor mudando os valores de ARR e CCRx com base em 1MHz do contador.
 *        Ex.: ARR = 1000000 e CCRx 1000000/2 (Sempre metade para que o duty cycle seja 50%)
 *        Assim temos: 1Hz = 1000000
 *                     2Hz = 500000
 *                     1KHz = 1000
 *                     2KHz = 500 Motor estável
 *                   2.5KHz = 400 Motor já não aceita alterações bruscas de velocidade
 *                     5KHz = 200
 *        Sabendo que o aparelho de experimento tem uma polia GT2 de 40 dentes de 2mm cada,
 *        motor de 200 passos por volta com o controlador fazendo 1/2 passo, resultando 400 passos
 *        por volta. 40 dentes * 2mm = 80mm por volta / 400 passos = 0.2mm por passo.
 *
 * @param Estrutura Pendulo_t
 * @param velocidade em mm/s
 */
static void Motor_Velocidade(Pendulo_t *p, uint16_t velocidade)
{
	uint32_t valor;
	if(velocidade > 0)
	{
		if(!(p->htim_motor_pwm->Instance->CCER & TIM_CCER_CC3E)) // PWM desativado
		{
			HAL_TIM_PWM_Start(p->htim_motor_pwm, p->canal_pwm);
		}
		valor = (uint32_t) ((200000 / velocidade) - 1.0f);
		p->htim_motor_pwm->Instance->ARR = valor;
		p->htim_motor_pwm->Instance->CCR3 = (uint32_t) valor / 2;
	} else
	{
		HAL_TIM_PWM_Stop(p->htim_motor_pwm, p->canal_pwm);
	}
}


// Selftest
static void Pendulo_SelfTest(Pendulo_t *p)
{
    p->estado = PENDULO_SWINGUP;
}

// Seta encoder
void Pendulo_PegaValorEncoder(Pendulo_t *p, int32_t encoder)
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
	HAL_TIM_PWM_Stop(p->htim_motor_pwm, p->canal_pwm);
    p->estado = PENDULO_ERRO;
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
    derivada = (erro - erro_anterior) / 0.001f; // 1ms

    p->pulso_motor =
            (p->kp * erro) +
            (p->ki * integral) +
            (p->kd * derivada);

    erro_anterior = erro;

    // maior que 30 graus volta para o Swing-UP
    if(fabsf(p->angulo) > 30.0f)
    {
        p->estado = PENDULO_SWINGUP;
    }
}


/**
 * @brief Calcula ângulo do pêndulo
 */
static void Pendulo_AtualizaAngulo(Pendulo_t *p)
{
    /*
     * Converter encoder em ângulo
     */

    p->angulo = (float)p->encoder / 27.777778f;
}


/**
 * @brief Velocidade angular do pêndulo
 */
static void Pendulo_AtualizaVelocidade(Pendulo_t *p)
{
    static float angulo_anterior = 0.0f;

    p->velocidade =
            (p->angulo - angulo_anterior) / 0.001f; // 1 ms

    angulo_anterior = p->angulo;
}


#include "penduloinvertido.h"

#include "math.h"
#include "stdlib.h"


// Variáveis internas
static float integral = 0.0f;
static float erro_anterior = 0.0f;
static EstadoChave_t EstadoChave;


// Funções privadas
static void SelfTest(Pendulo_t *p);
static void SwingUp(Pendulo_t *p);
static void Controle(Pendulo_t *p);
static void AtualizaAngulo(Pendulo_t *p);
static void AtualizaVelocidade(Pendulo_t *p);
static void VelocidadeMotor(Pendulo_t *p, int16_t velocidade);
static void Parar(Pendulo_t *p);
static EstadoChave_t ChaveFimCurso(Pendulo_t *p);
static void AtualizaPosicaoCarro(Pendulo_t *p);

/**
 * @brief Inicialização, basicamente passa as referências e inicia variáveis da estrutura
 */
void Pendulo_Inicializa(Pendulo_t *p, TIM_HandleTypeDef *htim_motor_pwm, uint32_t canal_pwm,
                        GPIO_TypeDef *chave_dir_port, uint16_t chave_dir_pin,
                        GPIO_TypeDef *chave_esq_port, uint16_t chave_esq_pin,
	                	GPIO_TypeDef *direcao_port, short unsigned int direcao_pin,
						TIM_HandleTypeDef *htim_conta_pulsos,
						TIM_HandleTypeDef *htim_encoder)
{
    p->encoder = 0;
    p->angulo = 0.0f;
    p->velocidade = 0.0f;
    p->posicao_carro = 0;
    p->pulso_motor = 0;

    p->kp = 0.8f;
    p->ki = 0.2f;
    p->kd = 0.1f;

    p->htim_motor_pwm = htim_motor_pwm;
    p->htim_conta_pulsos = htim_conta_pulsos;
    p->htim_encoder = htim_encoder;
    p->canal_pwm = canal_pwm;
    p->chave_dir_port = chave_dir_port;
    p->chave_dir_pin = chave_dir_pin;
    p->chave_esq_port = chave_esq_port;
    p->chave_esq_pin = chave_esq_pin;
    p->direcao_port = direcao_port;
    p->direcao_pin = direcao_pin;

    VelocidadeMotor(p, 0);

    p->estado = PENDULO_SELFTEST;
}


/**
 * @brief Atualiza pêndulo dentro dos seus estados.
 */
void Pendulo_AtualizaPendulo(Pendulo_t *p)
{
	// Evita crash do carro, exceto no SelfTest que tem um controle a parte
	if((ChaveFimCurso(p) == Chave_direita_fechada || ChaveFimCurso(p) == Chave_esquerda_fechada)
		&& (p->estado != PENDULO_SELFTEST))
	{
		p->estado = PENDULO_ERRO;
	}
    AtualizaAngulo(p);
    AtualizaVelocidade(p);
    AtualizaPosicaoCarro(p);

    switch(p->estado)
    {
        case PENDULO_SELFTEST:
            SelfTest(p);
        break;

        case PENDULO_SWINGUP:
            SwingUp(p);
            break;

        case PENDULO_CONTROLE:
            Controle(p);
            break;

        case PENDULO_ERRO:
            Parar(p);
            break;
    }
}


void AtualizaPosicaoCarro(Pendulo_t *p)
{
	p->posicao_carro = (int16_t) (p->htim_conta_pulsos->Instance->CNT - 8500);
}

/**
 * @brief Verifica estado das chaves de fim de curso
 */
static EstadoChave_t ChaveFimCurso(Pendulo_t *p)
{
	if(HAL_GPIO_ReadPin(p->chave_dir_port, p->chave_dir_pin) == GPIO_PIN_SET &&
	   HAL_GPIO_ReadPin(p->chave_esq_port, p->chave_esq_pin) == GPIO_PIN_SET)
	{
		EstadoChave = Chaves_abertas;
	}

	if(HAL_GPIO_ReadPin(p->chave_dir_port, p->chave_dir_pin) == GPIO_PIN_RESET)
	{
		EstadoChave = Chave_direita_fechada;
	}

	if(HAL_GPIO_ReadPin(p->chave_esq_port, p->chave_esq_pin) == GPIO_PIN_RESET)
	{
		EstadoChave = Chave_esquerda_fechada;
	}

	return EstadoChave;
}


/**
 * @brief Altera velocidade do motor mudando os valores de ARR e CCRx com base em 1MHz do contador.
 *        Ex.: ARR = 1000000 e CCRx 1000000/2 (Sempre metade para que o duty cycle seja 50%)
 *        Assim temos: 1Hz = 1000000
 *                     2Hz = 500000
 *                     1KHz = 1000
 *                     2KHz = 500 Motor estável
 *                   2.5KHz = 400 Motor já não aceita alterações bruscas de velocidade
 *
 *        Sabendo que o aparelho de experimento tem uma polia GT2 de 40 dentes de 2mm cada,
 *        motor de 200 passos por volta com o controlador fazendo 1/2 passo, resultando 400 passos
 *        por volta. 40 dentes * 2mm = 80mm por volta / 400 passos = 0.2mm por passo.
 *
 * @param Estrutura Pendulo_t
 * @param velocidade em mm/s
 * @param direcao 0 = Direita / 1 = Esquerda
 */
static void VelocidadeMotor(Pendulo_t *p, int16_t velocidade)
{
	uint32_t valor;
	static int16_t velocidade_anterior = 0;
	uint32_t contagem_anterior;

    // A T E N Ç Ã O: Verificar o tempo de acionamento (5us) da Direção do motor, tratar no início e no final
	// Analisar melhor, estou tratando somente o atraso depois da troca de direção e não antes
	// antes também tenho que desativar o pwm e ai sim alterar a direção.
    if(velocidade != velocidade_anterior)
    {
		if(velocidade > 0) // Carro para direira
		{
			if(HAL_GPIO_ReadPin(p->direcao_port, p->direcao_pin) == GPIO_PIN_SET) // Evita mudar sempre
			{
			    HAL_GPIO_WritePin(p->direcao_port, p->direcao_pin, 0); // Carro para direira
			    contagem_anterior = p->htim_conta_pulsos->Instance->CNT;
			    p->htim_conta_pulsos->Instance->CR1 &= ~TIM_CR1_DIR; // Contagem crescente
			    p->htim_conta_pulsos->Instance->CNT = contagem_anterior;
			    atraso_us(7); // Atraso necessário ao mudar de direção no DM556, visto no datasheet, mínimo de 5us
			}
		}
		else if (velocidade < 0)  // Carro para esquerda
		{
			if(HAL_GPIO_ReadPin(p->direcao_port, p->direcao_pin) == GPIO_PIN_RESET) // Evita mudar sempre
			{
			    HAL_GPIO_WritePin(p->direcao_port, p->direcao_pin, 1); // Carro para esquerda
			    contagem_anterior = p->htim_conta_pulsos->Instance->CNT;
			    p->htim_conta_pulsos->Instance->CR1 |= TIM_CR1_DIR; // Contagem decrescente
			    p->htim_conta_pulsos->Instance->CNT = contagem_anterior;
			    atraso_us(7); // Atraso necessário ao mudar de direção no DM556, visto no datasheet, mínimo de 5us
			}
		}

		if(velocidade != 0)
		{
			if(abs(velocidade) != abs(velocidade_anterior)) // Independente direita ou esquerda velocidade a mesma
				{
				if(!(p->htim_motor_pwm->Instance->CCER & TIM_CCER_CC3E)) // Evita ficar ativando PWM sempre
				{
					HAL_TIM_PWM_Start(p->htim_motor_pwm, p->canal_pwm);
				}
				valor = (uint32_t) ((200000 / abs(velocidade)) - 1.0f);
				if(p->htim_motor_pwm->Instance->CNT >= valor) //Para evitar problema do CNT ser maior que o ARR
				{
					p->htim_motor_pwm->Instance->CNT = 0;
				}
				p->htim_motor_pwm->Instance->ARR = valor;
				p->htim_motor_pwm->Instance->CCR3 = (uint32_t) valor / 2;
			}
		}
		else
		{
			if((p->htim_motor_pwm->Instance->CCER & TIM_CCER_CC3E)) // Evita ficar desativando PWM sempre
			{
			    HAL_TIM_PWM_Stop(p->htim_motor_pwm, p->canal_pwm);
			}
		}
    	velocidade_anterior = velocidade;
	}
}


// Selftest
static void SelfTest(Pendulo_t *p)
{
	int16_t velocidade;
	velocidade = 200; // mm/s

	p->estado = PENDULO_SWINGUP;

	if(ChaveFimCurso(p) == Chaves_abertas)
	{
		// Corre carro para direira
		VelocidadeMotor(p, velocidade);
	    while(ChaveFimCurso(p) != Chave_direita_fechada);
	    VelocidadeMotor(p, 0);
	    // Corre carro para esquerda
		VelocidadeMotor(p, -velocidade);
	    while(ChaveFimCurso(p) != Chave_esquerda_fechada);
	    VelocidadeMotor(p, 0);
	}

	if(ChaveFimCurso(p) == Chave_direita_fechada)
	{

	    // Corre carro para esquerda
		VelocidadeMotor(p, -velocidade);
	    while(ChaveFimCurso(p) != Chave_esquerda_fechada);

	    VelocidadeMotor(p, 0);
		// Corre carro para direira
		VelocidadeMotor(p, velocidade);
	    while(ChaveFimCurso(p) != Chave_direita_fechada);
	    VelocidadeMotor(p, 0);

	}

	if(ChaveFimCurso(p) == Chave_esquerda_fechada)
	{
		// Corre carro para direira
		VelocidadeMotor(p, velocidade);
	    while(ChaveFimCurso(p) != Chave_direita_fechada);
	    VelocidadeMotor(p, 0);
	}

    // Acertado o valor inicial do contador de pulsos
    p->htim_conta_pulsos->Instance->CNT = 10000;

    // Vai para o centro
    VelocidadeMotor(p, -velocidade);
    // Como o guia linear tem 600mm de área útil e se encontra na direita, retorna até 1500 pulsos a esquerda,
    // (cada puslo = 0.2mm) 1500 * 0,2mm = 300mm resultando o meio do barramento
    while(p->htim_conta_pulsos->Instance->CNT > 8500)
    {
    	if(ChaveFimCurso(p) == Chave_esquerda_fechada)
    	{
    		p->estado = PENDULO_ERRO;
    	}
    }
    VelocidadeMotor(p, 0);
	// CNT=8500 meio do barramento, a partir daqui a função "AtualizaPosicaoCarro"
	// Retorna -1500(Carro todo a esquerda) 0(Centro) +1500(Carro todo a direita)
}


// Atualiza Valor Encoder
void Pendulo_AtualizaValorEncoder(Pendulo_t *p, int32_t encoder)
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
void IniciaSwingUp(Pendulo_t *p)
{

	VelocidadeMotor(p, 0);
	AtualizaPosicaoCarro(p);

    p->estado = PENDULO_CONTROLE;
}


//Parar o pêndulo
void Parar(Pendulo_t *p)
{
	VelocidadeMotor(p, 0);
    p->estado = PENDULO_ERRO;
}

// SWING-UP
static void SwingUp(Pendulo_t *p)
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
static void Controle(Pendulo_t *p)
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
    tempo = p->pulso_motor;
    VelocidadeMotor(p, (int16_t) -p->pulso_motor);
    // maior que 30 graus volta para o Swing-UP
    if(fabsf(p->angulo) > 30.0f)
    {
    	VelocidadeMotor(p, 0);
        p->estado = PENDULO_SWINGUP;
    }
}


/**
 * @brief Calcula ângulo do pêndulo
 */
static void AtualizaAngulo(Pendulo_t *p)
{
    p->angulo = (float)p->encoder / 27.777778f; // 10000 contages em 360° = 27.7
    p->angulo = p->angulo - 180;
}


/**
 * @brief Velocidade angular do pêndulo
 */
static void AtualizaVelocidade(Pendulo_t *p)
{
    static float angulo_anterior = 0.0f;

    p->velocidade =
            (p->angulo - angulo_anterior) / 0.001f; // 1 ms

    angulo_anterior = p->angulo;
}

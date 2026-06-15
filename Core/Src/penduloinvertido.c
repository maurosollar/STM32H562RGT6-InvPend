
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
static void AtualizaVelocidadeAngularPendulo(Pendulo_t *p);
static void VelocidadeMotor(Pendulo_t *p, int16_t velocidade);
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
    p->angulo_pendulo = 0.0f;
    p->velocidade_angular_pendulo = 0.0f;
    p->posicao_carro = 0;
    p->velocidade_carro = 0;

    p->kp = 20.0f;
    p->ki = 0.0f;
    p->kd = 0.0001f;

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
	HAL_GPIO_WritePin(Time_Int_GPIO_Port, Time_Int_Pin, 1);

	AtualizaAngulo(p);
    AtualizaVelocidadeAngularPendulo(p);
    AtualizaPosicaoCarro(p);

	if((ChaveFimCurso(p) == Chave_direita_fechada || ChaveFimCurso(p) == Chave_esquerda_fechada)
		&& (p->estado != PENDULO_SELFTEST))
	{
		p->estado = PENDULO_ERRO;
	}


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
        	VelocidadeMotor(p, 0);
            p->estado = PENDULO_SELFTEST;
            break;
    }
    HAL_GPIO_WritePin(Time_Int_GPIO_Port, Time_Int_Pin, 0);
}


void AtualizaPosicaoCarro(Pendulo_t *p)
{
	p->posicao_carro = (int16_t) (p->htim_conta_pulsos->Instance->CNT - 20000) / 20;
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
 *        motor de 200 passos por volta com o controlador fazendo 1/8 passo, resultando 1600 passos
 *        por volta. 40 dentes * 2mm = 80mm por volta / 1600 passos = 0.05mm por passo.
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

	if(abs(velocidade) > 400)
	{
	    velocidade = (velocidade > 0) ? 400 : -400;
	}

	p->velocidade_carro = velocidade;

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
				//valor = (uint32_t) ((200000 / abs(velocidade)) - 1.0f); // Driver em 400 pulsos
				valor = (uint32_t) ((50000 / abs(velocidade)) - 1.0f); // Driver em 1600 pulsos
				//p->htim_motor_pwm->Instance->CNT = 0; // Verificar se é melhor deixar o CNT completar sua contagem para então zerar
				                                        // zerando ficou ruim
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
	}

	if(ChaveFimCurso(p) == Chave_direita_fechada)
	{
	    // Corre carro para esquerda até abrir a chave e desloca mais 100ms
		VelocidadeMotor(p, -velocidade);
	    while(ChaveFimCurso(p) != Chaves_abertas);
	    HAL_Delay(100);

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
    p->htim_conta_pulsos->Instance->CNT = 20000;

    // Vai para o centro
    VelocidadeMotor(p, -velocidade);
    // Como o guia linear tem 600mm de área útil e se encontra na direita, retorna até 1500 pulsos a esquerda,
    // (cada puslo = 0.05mm) 6000 * 0,05mm = 300mm resultando o meio do barramento
    while(p->htim_conta_pulsos->Instance->CNT > 14000)
    {
    	if(ChaveFimCurso(p) == Chave_esquerda_fechada)
    	{
    		p->estado = PENDULO_ERRO;
    	}
    }
    VelocidadeMotor(p, 0);
    // Assumindo área útil de segurança para a posição do carro são de 500mm, ficando 250mm para cada lado
    // Vamos zerar o carro aqui
    p->htim_conta_pulsos->Instance->CNT = 20000;
	// CNT=20000 meio do barramento, a partir daqui a função "AtualizaPosicaoCarro"
	// Retorna -5000(Carro todo a esquerda) 0(Centro) +5000(Carro todo a direita)
}


static void SwingUp(Pendulo_t *p)
{
    const float g = 9.81f;
    const float KSW = 10.0f;

    float theta;
    float omega;
    float E;
    float Ee;
    float comando;

    theta = p->angulo_pendulo * M_PI / 180.0f;
    omega = p->velocidade_angular_pendulo * M_PI / 180.0f;

    E =
        0.5f * omega * omega +
        g * (1.0f - cosf(theta));

    Ee = E - (2.0f * g);

    comando = KSW * Ee * omega * cosf(theta);

    if(comando > 400.0f)
        comando = 400.0f;

    if(comando < -400.0f)
        comando = -400.0f;

    VelocidadeMotor(p, (int16_t)comando);

    if(fabsf(p->angulo_pendulo) < 15.0f)
    {
        p->estado = PENDULO_CONTROLE;
    }
}

// Controle PID
static void Controle(Pendulo_t *p)
{
	const float dt = 0.001f;
    const float velocidade_max = 400.0f; // mm/s

    float erro;
    float derivada;
    float saida_pid;

    //erro = 0 - p->angulo_pendulo; // setpoint = 0°
    erro = (float) (((float) p->encoder) - 5000); // setpoint = 0°
    erro = erro + (p->posicao_carro/2); // Quanto maior o divisor mais instável

    // Integral
    integral += erro * dt;

    // Anti-windup
    if(integral > 100.0f)
        integral = 100.0f;

    if(integral < -100.0f)
        integral = -100.0f;

    // Derivada
    derivada = (erro - erro_anterior) / dt;

    // PID
    saida_pid =
            (p->kp * erro) +
            (p->ki * integral) +
            (p->kd * derivada);

    // Saturação da velocidade
    if(saida_pid > velocidade_max)
        saida_pid = velocidade_max;

    if(saida_pid < -velocidade_max)
        saida_pid = -velocidade_max;

    erro_anterior = erro;

    VelocidadeMotor(p, (int16_t)(saida_pid));

    // Sai da região de estabilização
    if(fabsf(p->angulo_pendulo) > 30.0f)
    {
        integral = 0.0f;
        erro_anterior = 0.0f;

        VelocidadeMotor(p, 0);

        p->estado = PENDULO_SWINGUP;
    }
}


void Pendulo_SetaPID(Pendulo_t *p, float kp, float ki, float kd)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
}


/**
 * @brief Calcula ângulo e velocidade do pêndulo
 */
static void AtualizaAngulo(Pendulo_t *p)
{
    p->encoder = p->htim_encoder->Instance->CNT;

    // Ângulo para controle (-180° a +180°)
    p->angulo_pendulo = ((float)p->encoder * 360.0f / 10000.0f) - 180.0f;
}

/**
 * @brief Calcula a velocidade angular do pêndulo usando janela de 5 ms
 */
static void AtualizaVelocidadeAngularPendulo(Pendulo_t *p)
{
    static uint16_t encoder_anterior = 0;
    static uint8_t  contador = 0;
    static uint8_t  primeira_leitura = 1;

    HAL_GPIO_WritePin(Time_Exec_GPIO_Port, Time_Exec_Pin, 1);

    if (primeira_leitura)
    {
        encoder_anterior  = p->encoder;
        p->velocidade_angular_pendulo = 0.0f;
        primeira_leitura = 0;

        HAL_GPIO_WritePin(Time_Exec_GPIO_Port, Time_Exec_Pin, 0);
        return;
    }

    contador++;
    if (contador < 5)
    {
    	encoder_anterior  = p->encoder;
        HAL_GPIO_WritePin(Time_Exec_GPIO_Port, Time_Exec_Pin, 0);
        return;
    }
    contador = 0;

    int32_t delta_encoder = (int32_t)p->encoder - (int32_t)encoder_anterior;

    // Corrige wrap-around
    if (delta_encoder >  5000)
    {
    	delta_encoder -= 10000;
    }
    else if (delta_encoder < -5000)
    {
    	delta_encoder += 10000;
    }

    p->velocidade_angular_pendulo = delta_encoder * 7.2f;

    encoder_anterior = p->encoder;

    HAL_GPIO_WritePin(Time_Exec_GPIO_Port, Time_Exec_Pin, 0);
}

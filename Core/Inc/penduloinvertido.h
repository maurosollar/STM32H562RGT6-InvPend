#ifndef PENDULOINVERTIDO_H_
#define PENDULOINVERTIDO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdint.h"

// Atenção, biblioteca exige os elementos abaixo no main.c
extern uint32_t tempo;
extern void atraso_us(uint32_t us);

typedef enum // Estados
{
	PENDULO_SELFTEST = 0,
    PENDULO_SWINGUP,
    PENDULO_CONTROLE,
    PENDULO_ERRO
} PenduloEstado_t;


typedef struct
{
    volatile uint32_t encoder;

    GPIO_TypeDef *chave_dir_port;
    uint16_t chave_dir_pin;

    GPIO_TypeDef *chave_esq_port;
    uint16_t chave_esq_pin;

    GPIO_TypeDef *direcao_port;
    short unsigned int direcao_pin;

    float angulo;
    float velocidade;
    float posicao_carro;

    uint32_t pulso_motor;

    float kp;
    float ki;
    float kd;

    TIM_HandleTypeDef *htim_motor_pwm;
    uint32_t canal_pwm;
    PenduloEstado_t estado;

} Pendulo_t;

typedef enum
{
	 Chaves_abertas = 0,
	 Chave_direita_fechada,
	 Chave_esquerda_fechada
} EstadoChave_t;

/**
 * @brief Protótipos das funções públicas
 */
void Pendulo_Inicializa(Pendulo_t *p, TIM_HandleTypeDef *htim_motor_pwm, uint32_t canal_pwm,
		                GPIO_TypeDef *chave_dir_port, uint16_t chave_dir_pin,
		                GPIO_TypeDef *chave_esq_port, uint16_t chave_esq_pin,
						GPIO_TypeDef *direcao_port, short unsigned int direcao_pin);
void Pendulo_AtualizaPendulo(Pendulo_t *pendulo);
void Pendulo_PegaValorEncoder(Pendulo_t *pendulo, int32_t encoder);
void Pendulo_SetaPID(Pendulo_t *pendulo, float kp, float ki, float kd);


#ifdef __cplusplus
}
#endif

#endif /* PENDULOINVERTIDO_H_ */

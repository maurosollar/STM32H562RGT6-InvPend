#ifndef PENDULOINVERTIDO_H_
#define PENDULOINVERTIDO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdint.h"

// Estados
typedef enum
{
    PENDULO_AUTOTESTE = 0,
    PENDULO_SWINGUP,
    PENDULO_CONTROLE,
    PENDULO_ERRO

} PenduloEstado_t;


// Estrutura principal
typedef struct
{
    volatile uint32_t encoder;

    float angulo;
    float velocidade;

    uint32_t pulso_motor;

    float kp;
    float ki;
    float kd;

    PenduloEstado_t estado;

} Pendulo_t;


// Funções públicas
void Pendulo_Inicializa(Pendulo_t *pendulo);
void Pendulo_Atualiza_1ms(Pendulo_t *pendulo);
void Pendulo_SetaEncoder(Pendulo_t *pendulo, int32_t encoder);
void Pendulo_SetaPID(Pendulo_t *pendulo,
                    float kp,
                    float ki,
                    float kd);

void Pendulo_IniciaSwingUp(Pendulo_t *pendulo);

void Pendulo_Parar(Pendulo_t *pendulo);

PenduloEstado_t Pendulo_PegaEstado(Pendulo_t *pendulo);

#ifdef __cplusplus
}
#endif

#endif /* PENDULOINVERTIDO_H_ */

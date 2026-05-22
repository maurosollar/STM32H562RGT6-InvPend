# STM32H562RGT6 - Pêndulo invertido

Projeto de controle de pêndulo invertido utilizando STM32H562RGT6.

## Documentação

- [Lista de materiais](./boom.txt)
- [Diagrama da placa STM32](./Docs/WeAct-STM32H5_64PIN-CoreBoard_V11%20SchDoc.pdf")

## Fotos

### STM32 / Driver  / Encoder de 2500 pulsos por revolução / Motor de passo

<p float="left">
    <img src="Docs/STM32H562RGT6.png" width="200">
    <img src="Docs/DM556.png" width="200">
    <img src="Docs/Encoder_Incremental.png" width="200">
    <img src="Docs/Motor_Passo_Nema23.png" width="200">    
</p>

### Início da juntada dos materiais

<img src="Docs/montagem01.png">


### Iniciando a configuração do STM32CubeMX

```
STMCubeMX Version 6.17.0
Start My project from MCU
  Access to mcu selector
  
  For better performance it is recommended to enable the instruction cache (ICACHE)
  and the MPU to access OTP & RO areas.
  Do you want to apply now such default configuration? (YES)
  
  Do you want to create a new project:
  (*) Without TrustZone activated?
  
  Aba Project Manager
    Project Name: STM32H562RGT6-InvPend
    Toolchain / IDE: STMCubeIDE
  
  Linker Settings
    Minimum Heap Size 0x200
    Minimum Stack Size 0x400
    
  Aba Pinout & Configuration
    System Core
      RCC
        HSE: Crystal/Ceramic Resonator
        LSE: Crystal/Ceramic Resonator
        
  Aba Clock Configuration
    Input frequency LSE: 32.768KHz
    Input frequency HSE: 8MHz
    HCLK(MHz): 250

```

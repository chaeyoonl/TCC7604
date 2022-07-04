
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2022 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "rb.h"
#include "rb.c"
#include "compareAndTrans.h"
#include "compareAndTransmit.c"
#include "calculateFunction.h"
// #include "sndFunction.h"
// #include "sndFunction.c"
#include <stdint.h>
#include <inttypes.h>

#include "ak7604.h"

#define SND_SOC_SPI 1
#define SND_SOC_I2C 2

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
RingFifo_t gtUart2Fifo;
__IO uint8_t wakeup_flag = 0;
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#ifdef __GNUC__

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  if (ch == '\n')
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r", 1, 0xFFFF);
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

static int XtiFsTab[] = {
    12288000, 18432000};

static int PLLInFsTab[] = {
    256000, 384000, 512000, 768000, 1024000,
    1152000, 1536000, 2048000, 2304000, 3072000,
    4096000, 4608000, 6144000, 8192000, 9216000,
    12288000, 18432000, 24576000};

static int sdfstab[] = {
    8000, 12000, 16000, 24000,
    32000, 48000, 96000};

static int sdbicktab[] = {
    64, 48, 32, 128, 256};

/* AK7604 Codec Private Data */
struct ak7604_priv
{
  int control_type;
  struct snd_soc_codec *codec;
  struct spi_device *spi;
  struct i2c_client *i2c;
  struct regmap *regmap;
  int fs;

  int pdn_gpio;
  int MIRNo;
  int status;

  int dresetn;
  int ckresetn; // 20180222

  int DSPPramMode;
  int DSPCramMode;
  int DSPOfregMode;

  int PLLInput; // 0 : XTI,   1 : BICK1 ...3 : BICK3
  int XtiFs;    // 0 : 12.288MHz,  1: 18.432MHz

  int SDfs[NUM_SYNCDOMAIN];   // 0:8kHz, 1:12kHz, 2:16kHz, 3:24kHz, 4:32kHz, 5:48kHz, 6:96kHz, 7:192kHz
  int SDBick[NUM_SYNCDOMAIN]; // 0:64fs, 1:48fs, 2:32fs, 3:128fs, 4:256fs

  int Master[NUM_SYNCDOMAIN]; // 0 : Slave Mode, 1 : Master
  int SDCks[NUM_SYNCDOMAIN];  // 0 : Low,  1: PLLMCLK,  2:XTI

  int TDMSDINbit[NUM_SDIO];
  int TDMSDOUTbit[NUM_SDIO];
  int DIEDGEbit[NUM_SDIO];
  int DOEDGEbit[NUM_SDIO];
  int DISLbit[NUM_SDIO];
  int DOSLbit[NUM_SDIO];

  int cramaddr;
  int cramcount;
  unsigned char cramvalue[48];
#ifdef AK7604_AUDIO_EFFECT
  int AE_A_Lch_dVol;
  int AE_A_Rch_dVol;
  int AE_D_Lch_dVol;
  int AE_D_Rch_dVol;
  int AE_IN_dVol;

  int AE_INT1_Lch_dVol;
  int AE_INT1_Rch_dVol;
  int AE_INT2_Lch_dVol;
  int AE_INT2_Rch_dVol;
  int AE_FL_dVol;
  int AE_FR_dVol;
  int AE_RL_dVol;
  int AE_RR_dVol;
  int AE_SWL_dVol;
  int AE_SWR_dVol;

  int AE_IN_mute;
  int AE_INT1_mute;
  int AE_INT2_mute;
  int AE_FRONT_mute;
  int AE_REAR_mute;
  int AE_SW_mute;

  int AE_F1_DeEmphasis;
  int AE_F2_BaseMidTreble;
  int AE_F2_BaseMidTreble_Sel;
  int AE_F3_BEX;
  int AE_F3_BEX_sel;
  int AE_F4_Compressor;
  int AE_F5_Surround;
  int AE_F6_Loudness;
  int AE_F6_Loudness_Flt_Sel;

  int AE_EQ_Mode;

  int AE_Delay_Mode;

  int AE_Input_Sel;
  int AE_SubWoofer_Sw;
  int AE_Function_Sw;

  int AE_MixerFL1;
  int AE_MixerFL2;
  int AE_MixerFL3;
  int AE_MixerFR1;
  int AE_MixerFR2;
  int AE_MixerFR3;
  int AE_MixerRL1;
  int AE_MixerRL2;
  int AE_MixerRL3;
  int AE_MixerRR1;
  int AE_MixerRR2;
  int AE_MixerRR3;

  int AE_MICnMusic_Lch_Sel;
  int AE_MICnMusic_Rch_Sel;

  int AE_DZF_Enable_Sw;
#endif
  int muteon;
  int cramwrite;
  int dsprun;
};

uint8_t i2cAddress = 0x38;

struct ak7604_priv ak7604;

/* USER CODE BEGIN 0 */
void SleepMode(void)
{

  /* Suspend SysTick */
  HAL_SuspendTick();

  /* Enable Power Peripheral */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Sleep Mode */
  HAL_PWR_EnterSLEEPMode(0, PWR_SLEEPENTRY_WFI);

  /* Resume SysTick When System Wake-up */
  HAL_ResumeTick();
}

void StopMode(void)
{

  /* Suspend SysTick */
  HAL_SuspendTick();

  /* Enable Power Peripheral */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* STOP Mode */
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);

  /* Resume SysTick When System Wake-up */
  HAL_ResumeTick();

  /* PLL Clock Recovery */
  SystemClock_Config();
}

void StandbyMode(void)
{

  /* Wait 5 seconds */
  HAL_Delay(100);

  /* Enable Power Peripheral */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Enter STANDBY mode */
  HAL_PWR_EnterSTANDBYMode();
}

// uint8_t snd_soc_read(uint16_t i2cAddress, uint16_t registerAddress, int addressSize)
// {

//   uint8_t i2cReadBuffer[16] = {
//       0,
//   };

//   // Byte Read
//   HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c2, i2cAddress, registerAddress, addressSize, i2cReadBuffer, 1, 1000);
//   HAL_Delay(1000);
//   if (ret == HAL_OK)
//   {
//     printf("success to read!! --> %X \n", i2cReadBuffer[0]);
//   }
//   else
//   {
//     printf(" Fail to read i2c !! I2C address -> %X, register address -> %X\n", i2cAddress, registerAddress);
//     if (ret == HAL_BUSY)
//       printf(" it's BUSY !!!  \n");
//     if (ret == HAL_ERROR)
//       printf(" it's ERROR !!!  \n");
//     if (ret == HAL_TIMEOUT)
//       printf(" it's TIMEOUT !!!  \n");
//   }

//   return i2cReadBuffer[0];
// }

uint8_t ak7604_i2c_read(uint8_t i2cAddress, unsigned char *tccData, int addressSize)
{
  HAL_StatusTypeDef ret = HAL_I2C_Master_Sequential_Transmit_IT(&hi2c2, i2cAddress, tccData, addressSize, I2C_FIRST_FRAME);

  HAL_Delay(10);

  while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY)
  {
  }

  uint8_t i2cReadBuffer[16] = {
      0,
  };

  HAL_StatusTypeDef rets = HAL_I2C_Master_Sequential_Receive_IT(&hi2c2, i2cAddress + 1, i2cReadBuffer, 1, I2C_LAST_FRAME);

  HAL_Delay(10);

  while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY)
  {
  }

  if (rets == HAL_OK)
  {
    printf("TCC7604 HAL_OK !! receive --> %X\n", i2cReadBuffer[0]);
    return i2cReadBuffer[0];
  }
  else
  {
    if (rets == HAL_BUSY)
      printf(" TCC7604 BUSY !!!  \n");
    if (rets == HAL_ERROR)
      printf(" TCC7604 ERROR !!!  \n");
    if (rets == HAL_TIMEOUT)
      printf(" TCC7604 TIMEOUT !!!  \n");
  }
}

uint8_t ak7604_i2c_write(uint16_t i2cAddress, uint8_t *tccData, int addressSize)
{
  HAL_StatusTypeDef ret = HAL_I2C_Master_Sequential_Transmit_IT(&hi2c2, i2cAddress, tccData, addressSize, I2C_FIRST_AND_LAST_FRAME);

  HAL_Delay(10);

  if (ret == HAL_OK)
  {
    printf("TCC7604 HAL_OK !! success to write !! --> %X, %X, %X, %X, %X\n", i2cAddress, tccData[0], tccData[1], tccData[2], tccData[3]);
  }
  else
  {
    if (ret == HAL_BUSY)
      printf(" TCC7604 BUSY !!!  \n");
    if (ret == HAL_ERROR)
      printf(" TCC7604 ERROR !!!  \n");
    if (ret == HAL_TIMEOUT)
      printf(" TCC7604 TIMEOUT !!!  \n");
  }
}
static bool ak7604_readable(struct device *dev, unsigned int reg)
{

  bool ret;

  if (reg == AK7604_03_RESERVED)
    ret = false;
  else if (reg <= AK7604_34_SDOUT3_FORMAT)
    ret = true;
  else if (reg < AK7604_50_INPUT_DATA)
    ret = false;
  else if (reg <= AK7604_53_STO_SETTING)
    ret = true;
  else if (reg < AK7604_60_DSP_SETTING1)
    ret = false;
  else if (reg <= AK7604_61_DSP_SETTING2)
    ret = true;
  else if (reg < AK7604_71_SRCMUTE_SETTING)
    ret = false;
  else if (reg <= AK7604_73_SRCFILTER_SETTING)
    ret = true;
  else if (reg < AK7604_81_MIC_SETTING)
    ret = false;
  else if (reg <= AK7604_8E_ADC_MUTEHPF)
    ret = true;
  else if (reg < AK7604_A1_POWERMANAGEMENT1)
    ret = false;
  else if (reg <= AK7604_A3_RESETCONTROL)
    ret = true;
  else if (reg < AK7604_C0_DEVICE_ID)
    ret = false;
  else if (reg <= AK7604_VIRT_CC_DSPOUT6_MIX)
    ret = true;
  else
    ret = true;

  return ret;
}

static bool ak7604_volatile(struct device *dev, unsigned int reg)
{
  bool ret;

#ifdef AK7604_DEBUG
  if (reg < AK7604_VIRT_REGISTER)
    ret = true;
  else
    ret = false;
#else
  if (reg < AK7604_C0_DEVICE_ID)
    ret = false;
  else if (reg < AK7604_VIRT_REGISTER)
    ret = true;
  else
    ret = false;
#endif
  return (ret);
}

uint8_t ak7604_write_register(uint8_t i2cAddress, unsigned int reg, unsigned int value)
{
  unsigned char tx[4];
  int wlen;
  uint8_t ret;

  wlen = 4;
  tx[0] = (unsigned char)COMMAND_WRITE_REG;
  tx[1] = (unsigned char)(0xFF & (reg >> 8));
  tx[2] = (unsigned char)(0xFF & reg);
  tx[3] = value;

  ret = ak7604_i2c_write(i2cAddress, tx, wlen);
  HAL_Delay(10);

  return ret;
}

unsigned int ak7604_read_register(uint8_t i2cAddress, unsigned int reg)
{
  unsigned char tx[3], rx[1];
  int wlen, rlen;
  int val, ret;
  unsigned int rdata;

  if (!ak7604_readable(NULL, reg))
    return 0xFF;

  wlen = 3;
  rlen = 1;
  tx[0] = (unsigned char)(COMMAND_READ_REG & 0x7F);
  tx[1] = (unsigned char)(0xFF & (reg >> 8));
  tx[2] = (unsigned char)(0xFF & reg);

  printf("%X, %X, %X, %X\n", i2cAddress, tx[0], tx[1], tx[2]);

  ret = ak7604_i2c_read(i2cAddress, tx, wlen);
  HAL_Delay(10);

  return ret;
}

int snd_soc_update_bits(uint8_t i2cAddress, unsigned short reg,
                        unsigned int mask, unsigned int value)
{
  bool change;
  unsigned int old, new;
  int ret;

  printf("=== it is snd_soc_update_bits function == \n");

  ret = ak7604_read_register(i2cAddress, reg);

  old = ret;
  new = (old & ~mask) | (value & mask);
  change = old != new;

  ak7604_write_register(i2cAddress, reg, new);

  if (ret < 0)
    return ret;

  return change;
}

int setPLLOut(uint8_t i2cAddress)
{
  int nPLLInFs;
  int value, nBfs, fs;
  int n, nMax;

  printf("======== setPLLOut ========\n\n");

  value = (ak7604.PLLInput << 5);
  snd_soc_update_bits(i2cAddress, AK7604_00_SYSTEMCLOCK_1, 0x60, value);

  if (ak7604.PLLInput == 0)
  {
    nPLLInFs = XtiFsTab[ak7604.XtiFs];
  }
  else
  {
    nBfs = sdbicktab[ak7604.SDBick[ak7604.PLLInput - 1]];
    fs = sdfstab[ak7604.SDfs[ak7604.PLLInput - 1]];
    nPLLInFs = nBfs * fs;
  }

  n = 0;
  nMax = sizeof(PLLInFsTab) / sizeof(PLLInFsTab[0]);

  do
  {
    if (nPLLInFs == PLLInFsTab[n])
      break;
    n++;
  } while (n < nMax);

  snd_soc_update_bits(i2cAddress, AK7604_00_SYSTEMCLOCK_1, 0x1F, n);

  return (0);
}

static int setSDClock(uint8_t i2cAddress, int nSDNo)
{
  printf("---- setSDClock -----\n\n");
  int addr;
  int fs, bickfs;
  int cksBickFs;
  int sdv, bdv;
  int csksd;

  if (nSDNo >= NUM_SYNCDOMAIN)
    return (0);

  fs = sdfstab[ak7604.SDfs[nSDNo]];
  bickfs = sdbicktab[ak7604.SDBick[nSDNo]] * fs;

  addr = AK7604_05_SYNCDOMAIN1_SET1 + (2 * nSDNo);

  switch (ak7604.SDCks[nSDNo])
  {
  case 3: // BICK1
  case 4: // BICK2
  case 5: // BICK3
    csksd = ak7604.SDCks[nSDNo] - 3;
    cksBickFs = sdbicktab[csksd] * sdfstab[ak7604.SDfs[csksd]];
    bdv = cksBickFs / bickfs;
    // if ( ( cksBickFs % bickfs ) != 0 ) {
    // 	printf("it's ERROR !! ---> cksBickFs = %d, bickfs = %d,  %d \n", cksBickFs, bickfs, cksBickFs % bickfs);
    //   }
    break;
  case 2: // XTI
    bdv = XtiFsTab[ak7604.XtiFs] / bickfs;
    break;
  default:
    bdv = 122880000 / bickfs;
    break;
  }

  sdv = ak7604.SDBick[nSDNo];
  if (nSDNo == 3)
    bdv = 9;
  // bdv--;
  if (bdv > 255)
  {
    printf("it's ERROR !! \n");
    bdv = 255;
  }

  snd_soc_update_bits(i2cAddress, addr, 0x07, sdv);
  addr++;
  ak7604_write_register(i2cAddress, addr, bdv);
  if (ak7604.PLLInput == (nSDNo + 1))
  {
    setPLLOut(i2cAddress);
  }
  return (0);
}

int ak7604_probe_edit()
{
  ak7604.control_type = SND_SOC_I2C;

  int compareTempDataEnd = 0;
  /* ak7604_porbe */

  uint16_t eepromAddress = 0xA0;
  int retss = 0;

  printf("\t %d , %d\n", ak7604.control_type, ak7604.pdn_gpio);

  // HAL_GPIO_WritePin(GPIOG, MCU_DSP_PD_Pin, GPIO_PIN_RESET);
  // HAL_Delay(10);

  int devid = ak7604_read_register(i2cAddress, AK7604_C0_DEVICE_ID); // --> 0xC0

  if (devid != 0x04)
  {
    printf("********* This is not AK7604! ********* --> %X\n", devid);
  }
  else
  {
    printf("success devid --> %X\n", devid);
  }

  ak7604.fs = 48000;
  ak7604.PLLInput = 0;
  ak7604.XtiFs = 0;
  ak7604.dresetn = 0;

  int nPLLInFs;
  int nBfs, fs;
  int nMax;
  int value = (ak7604.PLLInput << 5);

  printf("\n======= Set Clock !! =======\n\n");

  // snd_soc_update_bits(i2cAddress, AK7604_00_SYSTEMCLOCK_1, 0x60, value);  //System Clock Setting 1
  // --> 00 00 00

  if (ak7604.PLLInput == 0)
  {
    nPLLInFs = XtiFsTab[ak7604.XtiFs];
  }
  else
  {
    nBfs = sdbicktab[ak7604.SDBick[ak7604.PLLInput - 1]];
    fs = sdfstab[ak7604.SDfs[ak7604.PLLInput - 1]];
    nPLLInFs = nBfs * fs;
  }

  int n = 0;
  nMax = sizeof(PLLInFsTab) / sizeof(PLLInFsTab[0]);

  do
  {
    if (nPLLInFs == PLLInFsTab[n])
      break;
    n++;
  } while (n < nMax);

  if (n == nMax)
    printf("It's error !! \n");

  snd_soc_update_bits(i2cAddress, AK7604_00_SYSTEMCLOCK_1, 0x1F, n); // --> 00 00 0F

  for (int i = 0; i < (NUM_SYNCDOMAIN - 1); i++)
    ak7604.Master[i] = 0;
  ak7604.Master[3] = 1; // SYNC4 fixed master.

  for (int i = 0; i < NUM_SYNCDOMAIN; i++)
  {
    ak7604.SDBick[i] = 0; // 64fs
    ak7604.SDfs[i] = 5;   // 48kHz
    ak7604.SDCks[i] = 5;  // Low

    if (i == 3)
    {
      ak7604.SDBick[i] = 12; // 64fs
      ak7604.SDfs[i] = 5;    // 48kHz
      ak7604.SDCks[i] = 2;   // Low
    }

    setSDClock(i2cAddress, i);
  }

  for (int i = 0; i < NUM_SYNCDOMAIN; i++)
  {
    ak7604.TDMSDINbit[i] = 0;
    ak7604.TDMSDOUTbit[i] = 0;
    ak7604.DIEDGEbit[i] = 0;
    ak7604.DOEDGEbit[i] = 0;
    ak7604.DISLbit[i] = 0;
    ak7604.DOSLbit[i] = 0;
  }

  // DSP BANK Setting for Audio Effect.
  ak7604_write_register(i2cAddress, AK7604_60_DSP_SETTING1, 0x02); // DRMBK -> 2048/4096, DRMA -> Ring/Ring
  HAL_Delay(10);
  ak7604_write_register(i2cAddress, AK7604_A1_POWERMANAGEMENT1, 0xBC); // 30
  HAL_Delay(10);
  ak7604_write_register(i2cAddress, AK7604_01_SYSTEMCLOCK_2, 0x07); // DAC(1,2,3), ADC Clock Setting
  HAL_Delay(10);
  ak7604_write_register(i2cAddress, AK7604_0D_SDIN1_2_SYNC, 0x12); // SyncN -> LR/BICKN(if Master)
  HAL_Delay(10);
  ak7604_write_register(i2cAddress, AK7604_0E_SDIN3_4_SYNC, 0x30); // SyncN -> LR/BICKN(if Master)
  HAL_Delay(10);

  // //src clock setting
  // ak7604_write_register(i2cAddress, AK7604_17_SYNCDOMAIN_SEL9, 0x40);
  // HAL_Delay(10);

  ak7604_write_register(i2cAddress, AK7604_1F_DAC1_SELECT, 0x01);
  HAL_Delay(10);
  ak7604_write_register(i2cAddress, AK7604_20_DAC2_SELECT, 0x01);
  HAL_Delay(10);

  // //src select
  // ak7604_write_register(i2cAddress, AK7604_28_SRC1_SELECT, 0x01);
  // HAL_Delay(10);

  ak7604_write_register(i2cAddress, AK7604_12_SYNCDOMAIN_SEL4, 0x04);
  HAL_Delay(10);
  ak7604_write_register(i2cAddress, AK7604_53_STO_SETTING, 0x40);
  HAL_Delay(10);

  ak7604_write_register(i2cAddress, AK7604_02_SYSTEMCLOCK_3, 0x90);
  HAL_Delay(10);

  // //src mute
  // ak7604_write_register(i2cAddress, AK7604_71_SRCMUTE_SETTING, 0x00);
  // HAL_Delay(10);
  // ak7604_write_register(i2cAddress, AK7604_83_DAC1L_VOLUME, 0x00);
  // HAL_Delay(10);

  snd_soc_update_bits(i2cAddress, AK7604_89_DAC_MUTEFILTER, 0x70, 0x00); // DAC Soft Mute OFF (DAC1,2,3)
  HAL_Delay(10);

  // int testadbs = ak7604_read_register(i2cAddress, AK7604_89_DAC_MUTEFILTER);
  // printf("AK7604_89_DAC_MUTEFILTER --> %d\n", testadbs);

  //src select

  // ak7604_write_register(i2cAddress, AK7604_83_DAC1L_VOLUME, 0x00);
  // ak7604_write_register(i2cAddress, AK7604_84_DAC1R_VOLUME, 0x00);

  // int dac1lvolume = ak7604_read_register(i2cAddress, AK7604_83_DAC1L_VOLUME);
  // HAL_Delay(10);
  // printf("\n\n read dac1l volume --> %d \n", dac1lvolume);
  // int dac1rvolume = ak7604_read_register(i2cAddress, AK7604_84_DAC1R_VOLUME);
  // HAL_Delay(10);
  // printf("\n\n read dac1r volume --> %d \n", dac1rvolume);

  ak7604_write_register(i2cAddress, AK7604_A3_RESETCONTROL, 0x01);
  HAL_Delay(10);
  ak7604_write_register(i2cAddress, AK7604_A3_RESETCONTROL, 0x07);
  HAL_Delay(10);

  

  ak7604.MIRNo = 0;
  ak7604.status = POWERDOWN;

  ak7604.DSPPramMode = 0;
  ak7604.DSPCramMode = 0;

  ak7604.cramaddr = 0;
  ak7604.cramcount = 0;

#ifdef AK7604_AUDIO_EFFECT
  ak7604.AE_A_Lch_dVol = 121; // 0dB
  ak7604.AE_A_Rch_dVol = 121; // 0dB
  ak7604.AE_D_Lch_dVol = 121; // 0dB
  ak7604.AE_D_Rch_dVol = 121; // 0dB
  ak7604.AE_IN_dVol = 121;    // 0dB

  ak7604.AE_INT1_Lch_dVol = 121; // 0dB
  ak7604.AE_INT1_Rch_dVol = 121; // 0dB
  ak7604.AE_INT2_Lch_dVol = 121; // 0dB
  ak7604.AE_INT2_Rch_dVol = 121; // 0dB
  ak7604.AE_FL_dVol = 121;       // 0dB;
  ak7604.AE_FR_dVol = 121;       // 0dB;
  ak7604.AE_RL_dVol = 121;       // 0dB;
  ak7604.AE_RR_dVol = 121;       // 0dB;
  ak7604.AE_SWL_dVol = 121;      // 0dB;
  ak7604.AE_SWR_dVol = 121;      // 0dB;

  ak7604.AE_IN_mute = 0;    // off;
  ak7604.AE_INT1_mute = 0;  // off;;
  ak7604.AE_INT2_mute = 0;  // off;;
  ak7604.AE_FRONT_mute = 0; // off;;
  ak7604.AE_REAR_mute = 0;  // off;;
  ak7604.AE_SW_mute = 0;

  ak7604.AE_F1_DeEmphasis = 0;        // off
  ak7604.AE_F2_BaseMidTreble = 0;     // off
  ak7604.AE_F2_BaseMidTreble_Sel = 0; // off
  ak7604.AE_F3_BEX = 0;               // off
  ak7604.AE_F3_BEX_sel = 0;           // default
  ak7604.AE_F4_Compressor = 0;        // off
  ak7604.AE_F5_Surround = 0;          // off
  ak7604.AE_F6_Loudness = 0;          // off
  ak7604.AE_F6_Loudness_Flt_Sel = 0;  // pattern1

  ak7604.AE_EQ_Mode = 0; // Flat

  ak7604.AE_Delay_Mode = 0; // Default(off)

  ak7604.AE_Input_Sel = 1;    // Digital
  ak7604.AE_SubWoofer_Sw = 0; // Subwoofer
  ak7604.AE_Function_Sw = 1;  // on

  ak7604.AE_MixerFL1 = 121; // 0dB
  ak7604.AE_MixerFL2 = 0;   // Mute
  ak7604.AE_MixerFL3 = 0;   // Mute
  ak7604.AE_MixerFR1 = 121; // 0dB
  ak7604.AE_MixerFR2 = 0;   // Mute
  ak7604.AE_MixerFR3 = 0;   // Mute
  ak7604.AE_MixerRL1 = 121; // 0dB
  ak7604.AE_MixerRL2 = 0;   // Mute
  ak7604.AE_MixerRL3 = 0;   // Mute
  ak7604.AE_MixerRR1 = 121; // 0dB
  ak7604.AE_MixerRR2 = 0;   // Mute
  ak7604.AE_MixerRR3 = 0;   // Mute

  ak7604.AE_MICnMusic_Lch_Sel = 0; // Analog IN Lch
  ak7604.AE_MICnMusic_Rch_Sel = 0; // FL

  ak7604.AE_DZF_Enable_Sw = 0; // AllOff
#endif
  ak7604.cramwrite = 0;
  ak7604.dsprun = 0;
#ifdef AK7604_IO_CONTROL
  // init_ak7604_pd(ak7604);
#endif

  return 0;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint16_t AT24C256C_addr = 0xA0;
  uint8_t T_buffer[65];
  uint8_t R_buffer[65];
  uint8_t readBuffer[16];
  // uint8_t writteBuffer[16] = {0x05, 0x06, 0x07, 0x08, 0x09, 0x05, 0x06, 0x07, 0x08, 0x09, 0x05, 0x06, 0x07, 0x08, 0x09, 0x05};

  // uint8_t spiTransferBufferWrite[24] = {0x01, 0x02, 0x18, 0x14, 0x00, 0x80,
  //                                       0x32, 0x02, 0x43, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  //                                       0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x29};

  // uint8_t spiTransferBuffer[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  // uint8_t spiReciverBuffer[5] = {0x09, 0x09, 0x09, 0x09, 0x09};

  T_buffer[64] = 0x00;
  R_buffer[64] = 0x00;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  if (RB_init(&gtUart2Fifo, 16))
  {
  }

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  HAL_GPIO_WritePin(MAIN_ON_GPIO_Port, MAIN_ON_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SYS_ON_GPIO_Port, SYS_ON_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ACC_ON_GPIO_Port, ACC_ON_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  HAL_GPIO_WritePin(MCU_DSP_PD_GPIO_Port, MCU_DSP_PD_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  HAL_GPIO_WritePin(AMP_MUTE_MCU_GPIO_Port, AMP_MUTE_MCU_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 100);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 100);

  // writteBuffer[0] = 0x39;

  printf("=============\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t ch;
  int compareTempData = 0, compareCheck = 1, spiCheckInt = 0, compareTempDataEnd = 0, checkI2c = 1, checkDACSelect = 0, sendMsg = 1;
  
  while (1)
  {

    if (!RB_isempty(&gtUart2Fifo))
    { // RB_isempty -> ringbuffer empty
      ch = RB_read(&gtUart2Fifo);
      printf("%X \n", ch);

      if (compare(ch, compareTempData, compareCheck))
        (compareCheck == 1) ? compareTempData++ : compareTempDataEnd++;
    }

    if (compareTempData == 5 && sendMsg)
    {

      for (int i = 0; i < 600; i++)
      {
        uint8_t transmitDatas = returnTransmitData(i);
        printf("--> %X \n", transmitDatas);
        HAL_UART_Transmit(&huart2, (uint8_t *)&transmitDatas, 1, 0xFFFF);
      }

      compareTempData = 0;
      sendMsg--;
      compareCheck++;
    }

    if (compareTempDataEnd == 6 && checkI2c)
      checkI2c = ak7604_probe_edit();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /**Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure the Systick interrupt time
   */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
   */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 799;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 40;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 99;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 799;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 40;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 99;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);
}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 799;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 40;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 99;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, AMP_REMOTE_Pin | MAIN_ON_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SYS_ON_GPIO_Port, SYS_ON_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, ACC_ON_Pin | AMP_MUTE_MCU_Pin | MCU_DSP_PD_Pin | PWREN_AMP_Pin, GPIO_PIN_SET);
  // HAL_GPIO_WritePin(GPIOG, ACC_ON_Pin|MCU_DSP_PD_Pin|PWREN_AMP_Pin, GPIO_PIN_SET);
  // HAL_GPIO_WritePin(GPIOG, AMP_MUTE_MCU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, BOOT_MODE_Pin | MCU_TUNER1_RSTB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : AMP_REMOTE_Pin MAIN_ON_Pin */
  GPIO_InitStruct.Pin = AMP_REMOTE_Pin | MAIN_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ACC_DET_PA0_Pin */
  GPIO_InitStruct.Pin = ACC_DET_PA0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ACC_DET_PA0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ACC_DET_Pin */
  GPIO_InitStruct.Pin = ACC_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ACC_DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BU_DET_Pin */
  GPIO_InitStruct.Pin = BU_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BU_DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SYS_ON_Pin */
  GPIO_InitStruct.Pin = SYS_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SYS_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ACC_ON_Pin AMP_MUTE_MCU_Pin MCU_TUNER1_RSTB_Pin MCU_DSP_PD_Pin
                           PWREN_AMP_Pin */
  GPIO_InitStruct.Pin = ACC_ON_Pin | AMP_MUTE_MCU_Pin | MCU_TUNER1_RSTB_Pin | MCU_DSP_PD_Pin | PWREN_AMP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT_MODE_Pin */
  GPIO_InitStruct.Pin = BOOT_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOOT_MODE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BU_DET3_Pin */
  GPIO_InitStruct.Pin = BU_DET3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BU_DET3_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  uint8_t rx;

  if (UartHandle->Instance == USART2)
  {
    rx = (uint8_t)(UartHandle->Instance->DR & (uint8_t)0x00FF);
    RB_write(&gtUart2Fifo, rx);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  switch (GPIO_Pin)
  {

  case ACC_DET_Pin:
    wakeup_flag = 1;
    break;

  case BU_DET_Pin:
    printf("BU_DET_Pin interrupt!!\n");
    break;

  default:;
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

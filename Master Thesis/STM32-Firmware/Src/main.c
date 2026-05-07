/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   Totem-Pole PFC – clean control base with calibrated measurements
  * MCU:     STM32H755 (CM7)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
/* === Synchronization & AC-side logic ===
 *
 * pll_sogi.h      – PLL oparty o SOGI-QSG, estymuje kąt fazowy napięcia sieci
 * ac_commutation.h – logika komutacji strony AC (detekcja półokresu + dead-zone)
 */
#include "main.h"
#include "pll_sogi.h"
#include "ac_commutation.h"
#include "pi_controller.h"
//#include "pfc_current_ctrl.h"
//#include "pfc_voltage_ctrl.h"
//#include "recorder.h"
//#include "recorderDefines.h"
#include "pi_controller_current.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"
#include "stm32h7xx_hal_rcc.h"
#include <stdint.h>
#include <string.h>



PLL_SOGI_t pll;
PLL_SOGI_t pll_ma;
AC_Commutation_t ac_comm;
PI_Controller_current pic_i;
/* ========================================================================== */
/* === VOLTAGE LOOP (outer loop) =========================================== */
/* ========================================================================== */


PI_Controller   pi_v;

#define I_REF_AMP_MAX     30.0f   // [A] maksymalna amplituda prądu
#define I_REF_AMP_MIN      0.0f   // [A] minimum (PFC tylko pobiera, nie oddaje)
#define VOLTAGE_LOOP_DIV   10    // 10 kHz / 100 = 100 Hz pętli napięciowej
#define VDC_TARGET        400.0f   // [V] docelowe napięcie DC przy pełnej sieci
#define VAC_PEAK_TARGET   325.0f   // [V] amplituda sieci przy 230V RMS
#define VDC_VAC_RATIO     (VDC_TARGET / VAC_PEAK_TARGET)  // ≈ 1.2308

#define VDC_MIN           15.0f    // [V] minimalne vdc_ref (ochrona przy starcie)
#define VDC_MAX           405.0f   // [V] maksymalne vdc_ref (ochrona przed OVP)

volatile float vdc_ref = 0.0f;
volatile float i_ref_amp = 0.0f;    // wyjście PI napięciowego

static uint16_t v_loop_cnt = 0;



FDCAN_HandleTypeDef hfdcan2;
volatile uint8_t can_flag = 0;

/* ================= CAN DEBUG ================= */

volatile uint32_t can_cccr = 0;
volatile uint32_t can_psr  = 0;
volatile uint32_t can_ecr  = 0;
volatile uint32_t can_ir   = 0;
volatile uint32_t can_txfs = 0;
volatile uint32_t can_txbrp = 0;
volatile uint32_t can_txbar = 0;
volatile uint32_t can_txbtie = 0;
volatile uint32_t can_txbcf = 0;
volatile uint32_t can_txbc = 0;
volatile uint32_t can_ndat1 = 0;
volatile uint32_t can_ndat2 = 0;

volatile uint32_t can_tx_fifo_free = 0;

volatile uint32_t can_init_bit = 0;
volatile uint32_t can_busoff = 0;
volatile uint32_t can_error_passive = 0;
volatile uint32_t can_error_warning = 0;
volatile uint32_t can_last_error = 0;
volatile uint32_t can_tec = 0;
volatile uint32_t can_rec = 0;

volatile float can_rx_kp       = 0.0f;
volatile float can_rx_ki       = 0.0f;
volatile uint8_t can_gains_updated = 0;
volatile float can_rx_kp_v       = 0.0f;
volatile float can_rx_ki_v       = 0.0f;
volatile uint8_t can_vgains_updated = 0;
volatile float can_rx_vdc_ref  = 80.0f;




#define OUTER_LOOP_DIV  100     // 100 kHz / 100 = 1 kHz
static uint32_t outer_cnt = 0;
#define VAC_PEAK   230.0f   // 325 V
volatile float theta_test = 0.0f;


/* === Simulated inductor current (test mode) === */
static float iL_sim = 0.0f;
#define I_MODEL_K   0.05f     // szybkość odpowiedzi
#define DEG2RAD(x) ((x) * 3.1415926f / 180.0f)


/* ========================================================================== */
/* ========================= DEFINES ======================================== */
/* ========================================================================== */
#define I_TEST_AMP   1.0f   // [A] – mała, bezpieczna wartość

//#define I_REF_AMP   15.0f   // [A] — BEZPIECZNA wartość testowa
#define IL_OCP_LIMIT   10.0f   // [A] próg zwarciowy
#define VDC_OVP_LIMIT 427.0f


#define VREF 3.3f
#define TEST_GRID_FREQ   50.0f
#define TEST_VDC         400.0f
#define CTRL_TS          0.0001f // 10 us

/* =========================
 * SINUS RECORDING (PLL TEST)
 * ========================= */
/* =========================
 * SINUS RECORDING (RAW + FILTERED)
 * ========================= */

#define CTRL_DIV   10    // 100 kHz / 10 = 10 kHz

static uint8_t ctrl_div = 0;



#define SIN_REC_SAMPLES 4000
#define trzy_cztery_pi (3.0/2.0 * M_PI)
#define pi_pol (M_PI / 2.0)
#define DELTA_THETA 0.0314f   // ~1.8° // ≈ Δθ z artykułu
//#define DELTA_THETA   0.0f  // ≈ Δθ z artykułu
#define TWO_PI        (2.0f * M_PI)
#define PI_2          (0.5f * M_PI)
#define THREE_PI_2    (1.5f * M_PI)
#define DEAD_CNT  100
#define DELTA_OFF (M_PI / 6.0f)   // PRZED zerem (wyłączanie)
#define DELTA_ON    0.0f   // PO zerze (włączanie(M_PI / 125.0f))


volatile float C[SIN_REC_SAMPLES];
volatile float vac_filt_rec[SIN_REC_SAMPLES];
volatile float vac_raw_rec[SIN_REC_SAMPLES];

volatile uint16_t vac_rec_idx = 0;
volatile uint8_t  vac_rec_active = 0;
volatile uint8_t  vac_rec_done = 0;

/* ========================================================================== */
/* ========================= ADC DMA BUFFERS ================================= */
/* ========================================================================== */

/* ADC1: PA6, PA7, PF11 */
#define ADC1_DMA_SAMPLE_COUNT   25u
#define ADC1_DMA_CHANNELS       3u

volatile uint16_t adc1_raw[ADC1_DMA_CHANNELS * ADC1_DMA_SAMPLE_COUNT];

/* ADC2: CH5 diff, CH11 */
volatile uint16_t adc2_raw[2];

/* ========================================================================== */
/* ========================= SIGNALS ======================================== */
/* ========================================================================== */
float test_theta = 0.0f;
volatile uint16_t g_adc_pa6  = 0;
volatile uint16_t g_adc_pa7  = 0;
volatile uint16_t g_adc_pf11 = 0;
volatile uint16_t g_adc_vdc = 0;

volatile float vac_V = 0.0f;
volatile float vac_norm = 0.0f;
volatile float i_ref = 0.0f;
volatile float Vac_1 = 0.0f;
volatile float Vac_2 = 0.0f;
volatile float iL_A  = 0.0f;
volatile float vdc_V = 0.0f;
volatile float current_dbg = 0.0f;
volatile int testowa= 0.0;
volatile float d_ff = 0.0f;
volatile float d_ff_dbg = 0.0f;
volatile uint8_t ocp_latched = 0;
volatile uint8_t ovp_latched = 0;
volatile float d_pi = 0.0f;
int licznik=0;
volatile uint32_t control_tick = 0;
float skalar =0;
/* do debugowania w live expresions*/

volatile float i_ref_dbg = 0.0f;
volatile float sum_dbg = 0.0f;
volatile float iL_sim_dbg = 0.0f;
volatile float duty_dbg = 0.0f;
volatile float duty = 0.0f;
float voltage_ma=0.0f;
float voltage_out_ma=0.0f;
float voltage_out_ma_old=0.0f;
float current_ma=0.0f;
float voltage_ma_old=0.0f;
float current_ma_old=0.0f;


volatile float d_pi_dbg = 0.0f;
volatile int   ac_half_dbg = 0;
volatile int   ac_pwm_en_dbg = 0;
volatile float i_ref_amp_dbg = 0.0f;
volatile float vdc_err_dbg   = 0.0f;
volatile float vac_raw1_dbg     = 0.0f;
volatile float vac_raw2_dbg     = 0.0f;
volatile float vac_raw_dbg     = 0.0f;
volatile float vac_filt_dbg    = 0.0f;
volatile float v_alpha_dbg     = 0.0f;
volatile float v_beta_dbg      = 0.0f;
volatile float v_d_dbg         = 0.0f;
volatile float v_q_dbg         = 0.0f;
volatile float theta_dbg       = 0.0f;
volatile float voltage_matrix  = 0.0f;
volatile float sin_theta_dbg   = 0.0f;
volatile float cos_theta_ma_dbg   = 0.0f;
volatile float cos_theta_dbg   = 0.0f;
volatile float vac_norm_dbg    = 0.0f;
volatile float vac_norm_filtr_dbg    = 0.0f;
volatile float vac_raw1_dbg_norm = 0.0f;
volatile float vac_raw2_dbg_norm = 0.0f;
volatile float pll_omega_dbg   = 0.0f;
volatile float iL_dbg  = 0.0f;
volatile float vdc_dbg = 0.0f;
volatile int Fpos_dbg  = 0;
volatile int Fneg_dbg  = 0;
volatile int Fctrl_dbg = 0;

float Vac_ampl = 0.0f;
/* === Filtered AC voltage ===
 * Wykorzystywane jako wejście do PLL.
 */
volatile float vac_filt = 0.0f;

/* Współczynnik filtru IIR (0 < alpha < 1)
 * Mniejsza wartość = silniejsze tłumienie szumu,
 * ale większe opóźnienie sygnału.
 */
#define VAC_IIR_ALPHA  0.02f

/* =========================
   UART4 DEBUG LOGGER
   ========================= */

typedef struct
{
    float iL;
    float i_ref;
    float d_pi;
    float d_ff;
    float theta;
    float omega;
} DebugFrame_t;

volatile DebugFrame_t debug_frame;
volatile uint8_t debug_ready = 0;

typedef struct
{
    float d_pi;
    float d_ff;
    float i_ref;
    float iL_A;
    float i_ref_amp;
} CAN_DebugFrame_t;

volatile CAN_DebugFrame_t can_debug;
volatile uint8_t can_debug_ready = 0;







/* ========================================================================== */
/* ========================= MEASUREMENTS =================================== */
/* ========================================================================== */

/* PA6: U[V] = 0.00692 * ADC - 49.0 */
static inline float adc_pa6_to_vac(uint16_t adc)
{
  return 0.00692f * (float)adc - 49.0f;
}

/* PA7: U[V] = 0.00690 * ADC - 48.4 */
static inline float adc_pa7_to_vac(uint16_t adc)
{
  return 0.00690f * (float)adc - 48.4f;
}

/* VAC = PA6 - PA7 */
static inline float adc_vac_diff_to_V(uint16_t pa6, uint16_t pa7)
{
  return adc_pa6_to_vac(pa6) - adc_pa7_to_vac(pa7);
}


/* PF11: I[A] = 0.00103 * ADC - 34.7 */
static inline float adc_pf11_to_iL_A(uint16_t adc)
{
    return 0.001011f * (float)adc - 33.66f;
}

/* ADC2 CH5 diff: Udc[V] = 0.00502 * ADC - 167.1 */
static inline float adc_vdc_to_V(uint16_t adc)
{
    return 0.02127f * (float)adc - 707.6f;
}

/* ========================================================================== */
/* ========================= FILTERS =================================== */
/* ========================================================================== */

/* =========================
   VAC FILTER
   ========================= */
//static inline float iir_filter(float prev, float input, float alpha)
//{
//  return prev + alpha * (input - prev);
//}
void CAN_ProcessRx(void);

void filter(float *out,float *old, float coff, float in)
{
    *out= coff*in +(1-coff)*(*old);
    *old=*out;

}


void sprawdz(float theta)
{
    /* domyślnie wszystko OFF */
    Fpos_dbg = 0;
    Fneg_dbg = 0;


    /* ===== DEAD ZONE wokół zer cosinusa ===== */
    if (
        (theta >= (PI_2 - DELTA_THETA) && theta <= (PI_2 + DELTA_THETA)) ||
        (theta >= (THREE_PI_2 - DELTA_THETA) && theta <= (THREE_PI_2 + DELTA_THETA))
       )
    {
        return;
    }

    /* ===== COS > 0 → F_pos ===== */
    if (theta < (PI_2 - DELTA_THETA) ||
        theta > (THREE_PI_2 + DELTA_THETA))
    {
        Fpos_dbg = 1;

        return;
    }

    /* ===== COS < 0 → F_neg ===== */
    if (theta > (PI_2 + DELTA_THETA) &&
        theta < (THREE_PI_2 - DELTA_THETA))
    {
        Fneg_dbg = 1;

        return;
    }
}


static inline void calc_dff_ref(float theta)
{

    if (Fpos_dbg)
    {
        d_ff = 1.0f - (voltage_ma / voltage_out_ma);
    }
    else if (Fneg_dbg)
    {
       d_ff = - (voltage_ma/ voltage_out_ma);
    }
    else
    {
        d_ff = 0.0f;
    }
}

//static inline void calc_dff_ref(float theta)
//{
//    float cos_ref = cosf(theta);
//
//    if (Fpos_dbg)
//    {
//        d_ff = 1.0f - (cos_ref * VAC_PEAK / vdc_V);
//    }
//    else if (Fneg_dbg)
//    {
//        d_ff = - (cos_ref * VAC_PEAK / vdc_V);
//    }
//    else
//    {
//        d_ff = 0.0f;
//    }
//}


void CAN_ProcessRx(void)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0,
                                   &RxHeader, RxData) != HAL_OK)
            break;

        if (RxHeader.IdType     != FDCAN_STANDARD_ID  ||
            RxHeader.DataLength != FDCAN_DLC_BYTES_8)
            continue;

        /* ── regulator PRĄDU ── */
        if (RxHeader.Identifier == 0x400)
        {
            float kp, ki;
            memcpy(&kp, &RxData[0], 4);
            memcpy(&ki, &RxData[4], 4);

            if (kp >= 0.0f && kp <= 10.0f &&
                ki >= 0.0f && ki <= 1000.0f)
            {
                can_rx_kp = kp;
                can_rx_ki = ki;
                can_gains_updated = 1;
            }
        }

        /* ── regulator NAPIĘCIA ── */
        else if (RxHeader.Identifier == 0x401)
        {
            float kp, ki;
            memcpy(&kp, &RxData[0], 4);
            memcpy(&ki, &RxData[4], 4);

            if (kp >= 0.0f && kp <= 10.0f &&
                ki >= 0.0f && ki <= 1000.0f)
            {
                can_rx_kp_v = kp;
                can_rx_ki_v = ki;
                can_vgains_updated = 1;
            }
        }

        /* ── setpoint napięcia ── */
        else if (RxHeader.Identifier == 0x402)
        {
            float vref;
            memcpy(&vref, &RxData[0], 4);

            if (vref >= 0.0f && vref <= VDC_MAX)
            {
                can_rx_vdc_ref = vref;
            }
        }
    }
}





/* ========================================================================== */
/* ========================= PROTOTYPES ===================================== */
/* ========================================================================== */

void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_FDCAN2_Init(void);
//void CAN_Send_2Float(float a, float b);
//HAL_StatusTypeDef CAN_Send_2Float(float a, float b);
HAL_StatusTypeDef CAN_Send_2Float(uint16_t id, float a, float b);
HAL_StatusTypeDef CAN_Send_1Float(uint16_t id, float a);
/* =========================================================
 *  SWO / ITM support
 * ========================================================= */
static inline void SWO_SendFloat(uint8_t port, float value)
{
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) &&
        (ITM->TER & (1UL << port)))
    {
        ITM->PORT[port].u32 = *(uint32_t*)&value;
    }
}

void CAN_Debug_Read(void)
{
    can_cccr = hfdcan2.Instance->CCCR;
    can_psr  = hfdcan2.Instance->PSR;
    can_ecr  = hfdcan2.Instance->ECR;
    can_ir   = hfdcan2.Instance->IR;
    can_txfs = hfdcan2.Instance->TXFQS;
    can_txbrp = hfdcan2.Instance->TXBRP;
    can_txbar = hfdcan2.Instance->TXBAR;
    can_txbcf = hfdcan2.Instance->TXBCF;
    can_txbc = hfdcan2.Instance->TXBC;
    can_ndat1 = hfdcan2.Instance->NDAT1;
    can_ndat2 = hfdcan2.Instance->NDAT2;

    can_tx_fifo_free = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);

    /* Rozbicie bitów */
    can_init_bit      = (can_cccr & 0x1);
    can_busoff        = (can_psr >> 7) & 0x1;
    can_error_passive = (can_psr >> 6) & 0x1;
    can_error_warning = (can_psr >> 5) & 0x1;
    can_last_error    = (can_psr & 0x7);

    can_tec = (can_ecr >> 0) & 0xFF;
    can_rec = (can_ecr >> 8) & 0x7F;
}

/* ========================================================================== */
/* ========================= MAIN =========================================== */
/* ========================================================================== */

int main(void)
{
  HAL_Init();

  LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SYSCFG);

  SystemClock_Config();
  PeriphCommonClock_Config();

  MX_GPIO_Init();

  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_FDCAN2_Init();




  //TRACE_Init();



  /* =========================
       DMA CONFIG FOR ADC1
       ========================= */
  LL_DMA_SetPeriphAddress (DMA1, LL_DMA_STREAM_0, (uint32_t)&ADC1->DR);
  LL_DMA_SetMemoryAddress (DMA1, LL_DMA_STREAM_0, (uint32_t)adc1_raw);
  LL_DMA_SetDataLength    (DMA1, LL_DMA_STREAM_0,
                             ADC1_DMA_CHANNELS * ADC1_DMA_SAMPLE_COUNT);
  LL_DMA_EnableStream     (DMA1, LL_DMA_STREAM_0);

  ADC1->CFGR &= ~ADC_CFGR_DMNGT;
  ADC1->CFGR |=  (ADC_CFGR_DMNGT_0 | ADC_CFGR_DMNGT_1);

    /* =========================
       DMA CONFIG FOR ADC2
       ========================= */
  LL_DMA_SetPeriphAddress (DMA1, LL_DMA_STREAM_1, (uint32_t)&ADC2->DR);
  LL_DMA_SetMemoryAddress (DMA1, LL_DMA_STREAM_1, (uint32_t)adc2_raw);
  LL_DMA_SetDataLength    (DMA1, LL_DMA_STREAM_1, 2);
  LL_DMA_EnableStream     (DMA1, LL_DMA_STREAM_1);

  ADC2->CFGR &= ~ADC_CFGR_DMNGT;
  ADC2->CFGR |=  (ADC_CFGR_DMNGT_0 | ADC_CFGR_DMNGT_1);

  /* ===== NVIC FOR TIM1 ===== */
  NVIC_SetPriority(TIM1_UP_IRQn, 0);
  NVIC_EnableIRQ(TIM1_UP_IRQn);

  /* PWM start */
  //LL_TIM_OC_SetCompareCH1(TIM1, 0.5*2399);   // 50% przy ARR=4799
  LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
  LL_TIM_DisableAllOutputs(TIM1);        //
  LL_TIM_EnableCounter(TIM1);

  /* ADC start */
  LL_ADC_Enable(ADC1);
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC1)) {}
  LL_ADC_REG_StartConversion(ADC1);

  LL_ADC_Enable(ADC2);
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC2)) {}
  LL_ADC_REG_StartConversion(ADC2);

  /* --- PLL init (50 Hz) --- */
  PLL_SOGI_Init(&pll, 314.15927f);   // 2*pi*50
  PLL_SOGI_Init(&pll_ma, 314.15927f);


  /* Control loop */
  PIC_Init(&pic_i,
           0.014f,     // Kp
           20.0f,     // Ki
           0.0001f,   // Ts = 10 kHz
           -0.45f,     // out_min
            0.45f);    // out_max

  /* Regulator napięcia DC (pętla zewnętrzna, 100 Hz) */
  PI_Init(&pi_v,
          0.003f,          // Kp  – MAŁA wartość startowa!
          0.2f,           // Ki
          0.001f,          // Ts = 10 ms (100 Hz)
          I_REF_AMP_MIN,
          I_REF_AMP_MAX);


  /* Control loop sampling:
   * TIM1 update = 100 kHz
   * Ts = 10 µs
   */
//  PFC_CurrentCtrl_Init(&pfc_i,
//      0.5f,      /* Kp */
//      30.0f,     /* Ki */
//      0.00001f,  /* Ts = 10 µs */
//      -1.0f,
//      0.95f);
//  pfc_i.test_mode = false;
//
//  /* === Voltage loop init (outer loop) ===
//   * Fs = 1 kHz  → Ts = 1 ms
//   */
//  PFC_VoltageCtrl_Init(&pfc_v,
//      0.05f,     /* Kpv  (START SAFE) */
//      10.0f,     /* Kiv */
//      0.001f,    /* Ts = 1 ms */
//      0.0f,      /* I_ref min */
//      5.0f       /* I_ref max [A] */
//  );


  while (1)
  {
      if (can_debug_ready)
      {
          can_debug_ready = 0;
          if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) >= 3)
          {
              CAN_Send_2Float(0x321, can_debug.d_pi,  can_debug.d_ff);
              CAN_Send_2Float(0x322, can_debug.i_ref, can_debug.iL_A);
              CAN_Send_1Float(0x323, can_debug.i_ref_amp);
          }
      }

      CAN_ProcessRx();

      /* regulator prądu */
      if (can_gains_updated)
      {
          can_gains_updated = 0;
          PIC_ChangeNastawa(&pic_i, can_rx_kp, can_rx_ki);
      }

      /* regulator napięcia ← NOWE */
      if (can_vgains_updated)
      {
          can_vgains_updated = 0;
          PI_ChangeNastawa(&pi_v, can_rx_kp_v, can_rx_ki_v);
      }
  }
}
/* ========================================================================== */
/* ========================= TIM1 ISR ======================================= */
/* ========================================================================== */

void TIM2_IRQHandler(void)
{
	if (!LL_TIM_IsActiveFlag_UPDATE(TIM2))
	        return;

	    LL_TIM_ClearFlag_UPDATE(TIM2);

	    //if (ocp_latched)
	    		//return;



	    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_6);



	    /* --- tu właściwy kod ISR --- */

   /* =====================================================
    * 1. ADC – OSTATNIA PRÓBKA Z DMA
    * ===================================================== */
    uint32_t idx = (ADC1_DMA_SAMPLE_COUNT - 1) * ADC1_DMA_CHANNELS;

    g_adc_pa6  = adc1_raw[idx + 0];
    g_adc_pa7  = adc1_raw[idx + 1];
    g_adc_pf11 = adc1_raw[idx + 2];
    g_adc_vdc  = adc2_raw[0];




    /* === DEBUG === */




    /* =====================================================
     * 2. VAC → VOLTY
     * ===================================================== */
    vac_V = adc_vac_diff_to_V(g_adc_pa6, g_adc_pa7);

    filter(&voltage_ma,&voltage_ma_old,0.2f,vac_V);

//    vac_raw_dbg = vac_V;
//    Vac_1 = adc_pa7_to_vac(g_adc_pa7);
//    vac_raw1_dbg = Vac_1;
//    vac_raw1_dbg_norm = Vac_1/VAC_PEAK;
//    Vac_2 = adc_pa6_to_vac(g_adc_pa6);
//    vac_raw2_dbg = Vac_2;
//    vac_raw2_dbg_norm = Vac_2/VAC_PEAK;

   /* =====================================================
     * 3. FILTR IIR (ANTI-SZUM + OGRANICZENIE dV/dt)
     * ===================================================== */
    //vac_filt = iir_filter(vac_filt, vac_V, VAC_IIR_ALPHA);
    //vac_filt_dbg = voltage_ma;
    //current_dbg = current_ma;

    /* PRĄD – TYLKO POMIAR */
    iL_A  = adc_pf11_to_iL_A(g_adc_pf11);
    filter(&current_ma, &current_ma_old, 0.2f, iL_A);

    //current_dbg = current_ma;

    /* VDC – TYLKO POMIAR */
    vdc_V = adc_vdc_to_V(g_adc_vdc);
    filter(&voltage_out_ma, &voltage_out_ma_old, 0.1f, vdc_V);
   // vdc_dbg = vdc_V;



        /* === PLL === */
    //PLL_SOGI_Step(&pll, vac_V);
    PLL_SOGI_Step(&pll, vac_V);

    PLL_SOGI_Step(&pll_ma, voltage_ma);


    /* === IDEALNY TESTOWY KĄT === */
//    theta_test += 2.0f * M_PI * 50.0f * CTRL_TS;   // 50 Hz
//    if (theta_test >= TWO_PI)
//        theta_test -= TWO_PI;
//
//    pll.theta = theta_test;

        /* === AC COMMUTATION + d_ff === */
    sprawdz(pll.theta);
    calc_dff_ref(pll.theta);


    			//theta_dbg     = pll.theta;
                  // pll_omega_dbg = pll.omega;
                  // d_ff_dbg      = d_ff;

                   sin_theta_dbg = sinf(pll_ma.theta);
                   cos_theta_ma_dbg = cosf(pll_ma.theta);
                   cos_theta_dbg = cosf(pll.theta);


                  // v_d_dbg       = pll.v_d;
                 //  v_q_dbg       = pll.v_q;
                  // vac_norm_filtr_dbg  = vac_filt / VAC_PEAK;
                   //vac_norm_dbg  = vac_norm;


                   //if (cos_theta_dbg>0.005 ||cos_theta_dbg<-0.005 )skalar =voltage_ma/cos_theta_dbg;
                   //else skalar=1;
                   Vac_ampl = (pll_ma.u_mod > 8.0f) ? pll_ma.u_mod : 8.0f;
                   voltage_matrix = voltage_ma*0.987688341f - (Vac_ampl*sin_theta_dbg*0.156434465f);

 /* =====================================================
* PĘTLA NAPIĘCIOWA – 100 Hz (co 100 taktów 10 kHz)
* ===================================================== */

   if (++v_loop_cnt >= VOLTAGE_LOOP_DIV)
   {
	   v_loop_cnt = 0;
	   //vdc_ref = can_rx_vdc_ref;
	   vdc_ref = Vac_ampl * VDC_VAC_RATIO*1.32;

	   /* PI napięciowy: setpoint=vdc_ref, pomiar=vdc_V
		* voltage_in = vac_V (używane do warunkowego działania w PI_Update)
		* Wyjście: amplituda referencji prądowej */
	   if (vdc_ref < VDC_MIN) vdc_ref = VDC_MIN;
	   if (vdc_ref > VDC_MAX) vdc_ref = VDC_MAX;

	   i_ref_amp = PI_Update(&pi_v, vdc_ref, voltage_out_ma);
   }

        /* =====================================================
           * 7. PĘTLA PRĄDOWA (PI)
           * ===================================================== */
    vac_norm  = voltage_ma / Vac_ampl;
    i_ref = i_ref_amp * vac_norm;
    //i_ref = 10.0f * vac_norm;
    if (Fpos_dbg || Fneg_dbg)
    {
    	d_pi = PIC_Update(&pic_i, i_ref, iL_A);
    }


    d_pi_dbg = d_pi;
    //i_ref_dbg = i_ref;

          /* =====================================================
              * 8. SUMA + SATURACJA
              * ===================================================== */

//    if (d_ff > 0.5f) d_ff = 0.5f;
//	if (d_ff < 0.0f)  d_ff = 0.0f;



    if (Fpos_dbg)
        {

            if(d_ff>1)d_ff=1;
            if(d_ff<0.25f)d_ff=0.25f;
        }
        else if (Fneg_dbg)
        {


        	 if (d_ff > 0.75f) d_ff = 0.75f;
        	if (d_ff < 0.0f)  d_ff = 0.0f;
        }

    duty = d_pi+ d_ff;
    //duty = d_ff;

    d_ff_dbg = d_ff;








    //duty_dbg = duty;

    //recorderStep();
    if (vdc_V < 15.0f)
       	    {
       	    	ovp_latched = 1;
       	    	PI_Reset(&pi_v);           // ← dodaj
       	    	i_ref_amp = 0.0f;
       	    	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);

       	        return;
       	    }
       /* Zabezpieczenie prądowe */
       if ((vdc_V) > VDC_OVP_LIMIT)
      {
           ocp_latched = 1;
           LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
           return;
       }

       if (ocp_latched==1)return;

       ovp_latched = 0;
       ocp_latched = 0;

       if (!Fpos_dbg && !Fneg_dbg)
       {
    	   LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);

    	   static uint16_t can_div = 0;

    	   if (++can_div >= 2)   // 10 kHz / 2 = 5 kHz
    	   {
    	       can_div = 0;

    	       can_debug.d_pi = d_pi;
    	       can_debug.d_ff = duty;
    	       can_debug.i_ref = i_ref;
    	       can_debug.iL_A = iL_A;
    	       can_debug.i_ref_amp = voltage_ma;

    	       can_debug_ready = 1;
    	   }
    	   return;
       }

             /* =====================================================
              * 9. PWM → TIM1
              * ===================================================== */
   LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
   LL_TIM_OC_SetCompareCH1(TIM1, (uint32_t)(duty*2399));


   /* =========================
      UART DEBUG (1 kHz)
      ========================= */
   /* =========================
      CAN DEBUG 5 kHz
      ========================= */





   static uint16_t can_div = 0;

   if (++can_div >= 2)   // 10 kHz / 2 = 5 kHz
   {
       can_div = 0;

       can_debug.d_pi = d_pi;
       can_debug.d_ff = duty;
       can_debug.i_ref = i_ref;
	   can_debug.iL_A = iL_A;
	   can_debug.i_ref_amp = voltage_ma;

       can_debug_ready = 1;
   }
   LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6);

    /* =====================================================
        * wyłaczanie PWM i liczenia podczas deadzone
        * ===================================================== */

//    uint8_t dead_zone = (Fpos_dbg == 0 && Fneg_dbg == 0);
//
//    if (!dead_zone)
//    {
//        float d_pi = PIC_Update(&pic_i, i_ref, iL_A);
//        float duty = d_ff + d_pi;
//
//        if (duty > 0.95f) duty = 0.95f;
//        if (duty < 0.0f)  duty = 0.0f;
//
//        LL_TIM_OC_SetCompareCH1(TIM1, duty * PWM_PERIOD);
//        LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
//    }
//    else
//    {
//        LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
//    }

 /* === DO DEBUGOWANIA=== */

       // LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6);



    /* =========================
     * TRACE OUTPUT (10 kHz)
     * ========================= */
    //static uint32_t trace_div = 0;

    //if (++trace_div >= 10)   // 100 kHz / 10 = 10 kHz
    //{
    //   trace_div = 0;

    //    TRACE_SendFloat(0, vac_V);      // CH0
    //    TRACE_SendFloat(1, vac_filt);          // CH1
    //    TRACE_SendFloat(2, vdc_V);         // CH2
    //    TRACE_SendFloat(3, pll.theta);     // CH3
 //   }


    /* =====================================================
     * 6. BEZPIECZEŃSTWO – PWM WYŁĄCZONE
     * ===================================================== */

}

//void CAN_Send_2Float(float a, float b)
//{
//    FDCAN_TxHeaderTypeDef TxHeader;
//    uint8_t data[8];
//
//    TxHeader.Identifier = 0x321;
//    TxHeader.IdType = FDCAN_STANDARD_ID;
//    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
//    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
//    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
//    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
//    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//    TxHeader.MessageMarker = 0;
//
//    memcpy(&data[0], &a, 4);
//    memcpy(&data[4], &b, 4);
//
//    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0)
//    {
//        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, data) != HAL_OK)
//        {
//            Error_Handler();
//        }
//    }
//}

//HAL_StatusTypeDef CAN_Send_2Float(float a, float b)
//{
//    FDCAN_TxHeaderTypeDef TxHeader;
//    uint8_t data[8];
//
//    TxHeader.Identifier = 0x321;
//    TxHeader.IdType = FDCAN_STANDARD_ID;
//    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
//    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
//    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
//    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
//    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//    TxHeader.MessageMarker = 0;
//
//    memcpy(&data[0], &a, 4);
//    memcpy(&data[4], &b, 4);
//
//    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, data);
//}
HAL_StatusTypeDef CAN_Send_2Float(uint16_t id, float a, float b)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t data[8];

    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    memcpy(&data[0], &a, 4);
    memcpy(&data[4], &b, 4);

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, data);
}

HAL_StatusTypeDef CAN_Send_1Float(uint16_t id, float a)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t data[8] = {0};

    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;   // <--- zmiana
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    memcpy(&data[0], &a, 4);

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, data);
}
/* ========================================================================== */
/* ========================= INIT FUNCTIONS ================================= */
/* ========================================================================== */

/* ⚠️ PONIŻEJ WKLEJONE 1:1 Z TWOJEGO DZIAŁAJĄCEGO KODU */
/* ADC1, ADC2, TIM1, DMA, GPIO – BEZ ZMIAN LOGIKI */

/* --- TU WKLEJ SWOJE:
   MX_GPIO_Init
   MX_DMA_Init
   MX_ADC1_Init
   MX_ADC2_Init
   MX_TIM1_Init
   (DOKŁADNIE TAKIE JAK W PIERWSZYM main.c)
*/
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4) {}

  LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE0);
  while (LL_PWR_IsActiveFlag_VOS() == 0) {}

  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1) {}

  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_HSI_SetDivider(LL_RCC_HSI_DIV1);
  LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSI);
  LL_RCC_PLL1P_Enable();
  LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
  LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL1_SetM(4);
  LL_RCC_PLL1_SetN(60);
  LL_RCC_PLL1_SetP(2);
  LL_RCC_PLL1_SetQ(8);
  LL_RCC_PLL1_SetR(2);
  LL_RCC_PLL1_Enable();
  LL_RCC_PLL1Q_Enable();

  while(LL_RCC_PLL1_IsReady() != 1) {}

  LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1) {}

  LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_2);
  LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_2);

  LL_Init1msTick(480000000);
  LL_SetSystemCoreClock(480000000);
}

void PeriphCommonClock_Config(void)
{
  LL_RCC_PLL2P_Enable();
  LL_RCC_PLL2_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_2_4);
  LL_RCC_PLL2_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL2_SetM(32);
  LL_RCC_PLL2_SetN(100);
  LL_RCC_PLL2_SetP(4);
  LL_RCC_PLL2_SetQ(2);
  LL_RCC_PLL2_SetR(2);
  LL_RCC_PLL2_Enable();

  while(LL_RCC_PLL2_IsReady() != 1) {}
  LL_RCC_SetFDCANClockSource(LL_RCC_FDCAN_CLKSOURCE_PLL1Q);
}

static void MX_ADC1_Init(void)
{
  LL_ADC_InitTypeDef        ADC_InitStruct      = {0};
  LL_ADC_REG_InitTypeDef    ADC_REG_InitStruct  = {0};
  LL_ADC_CommonInitTypeDef  ADC_CommonInitStruct= {0};
  LL_GPIO_InitTypeDef       GPIO_InitStruct     = {0};

  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_PLL2P);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC12);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOF);

  /* PA6, PA7 */
  GPIO_InitStruct.Pin  = LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PF11 */
  GPIO_InitStruct.Pin  = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  LL_DMA_SetPeriphRequest         (DMA1, LL_DMA_STREAM_0, LL_DMAMUX1_REQ_ADC1);
  LL_DMA_SetDataTransferDirection (DMA1, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetStreamPriorityLevel   (DMA1, LL_DMA_STREAM_0, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode                  (DMA1, LL_DMA_STREAM_0, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetPeriphIncMode         (DMA1, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode         (DMA1, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize            (DMA1, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_HALFWORD);
  LL_DMA_SetMemorySize            (DMA1, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_HALFWORD);
  LL_DMA_DisableFifoMode          (DMA1, LL_DMA_STREAM_0);

  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  ADC_InitStruct.Resolution   = LL_ADC_RESOLUTION_16B;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);

  ADC_REG_InitStruct.TriggerSource   = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS;
  ADC_REG_InitStruct.SequencerDiscont= DISABLE;
  ADC_REG_InitStruct.ContinuousMode  = LL_ADC_REG_CONV_CONTINUOUS;
  ADC_REG_InitStruct.Overrun         = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);

  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV2;
  ADC_CommonInitStruct.Multimode   = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

  LL_ADC_DisableDeepPowerDown(ADC1);
  LL_ADC_EnableInternalRegulator(ADC1);
  __IO uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0) { wait_loop_index--; }

  LL_ADC_REG_SetSequencerRanks  (ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_3);
  LL_ADC_SetChannelSamplingTime (ADC1, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_32CYCLES_5);
  LL_ADC_SetChannelSingleDiff   (ADC1, LL_ADC_CHANNEL_3, LL_ADC_SINGLE_ENDED);

  LL_ADC_REG_SetSequencerRanks  (ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_7);
  LL_ADC_SetChannelSamplingTime (ADC1, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_32CYCLES_5);
  LL_ADC_SetChannelSingleDiff   (ADC1, LL_ADC_CHANNEL_7, LL_ADC_SINGLE_ENDED);

  LL_ADC_REG_SetSequencerRanks  (ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_2);
  LL_ADC_SetChannelSamplingTime (ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_32CYCLES_5);
  LL_ADC_SetChannelSingleDiff   (ADC1, LL_ADC_CHANNEL_2, LL_ADC_SINGLE_ENDED);

  ADC1->PCSEL = (1U << 2) | (1U << 3) | (1U << 7);
}


static void MX_FDCAN2_Init(void)
{


    __HAL_RCC_FDCAN_CLK_ENABLE();

    hfdcan2.Instance = FDCAN2;
    hfdcan2.Init.TxEventsNbr = 0;
    hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;  // testowo
    hfdcan2.Init.AutoRetransmission = ENABLE;
    hfdcan2.Init.TransmitPause = DISABLE;
    hfdcan2.Init.ProtocolException = DISABLE;

    hfdcan2.Init.NominalPrescaler = 12;
    hfdcan2.Init.NominalSyncJumpWidth = 1;
    hfdcan2.Init.NominalTimeSeg1 = 16;
    hfdcan2.Init.NominalTimeSeg2 = 3;

    hfdcan2.Init.MessageRAMOffset = 0;

    hfdcan2.Init.StdFiltersNbr = 1;
    hfdcan2.Init.ExtFiltersNbr = 0;

    hfdcan2.Init.RxFifo0ElmtsNbr = 4;
    hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan2.Init.TxBuffersNbr = 0;
    hfdcan2.Init.TxEventsNbr = 0;
    hfdcan2.Init.RxBuffersNbr = 0;

    hfdcan2.Init.TxFifoQueueElmtsNbr = 4;
    hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;

    if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
        Error_Handler();

    FDCAN_FilterTypeDef sFilterConfig = {0};

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;

    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
        Error_Handler();

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE) != HAL_OK)
        Error_Handler();

    // START NA KOŃCU
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
        Error_Handler();
}


static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_PLL2P);

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC12);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  /**ADC2 GPIO Configuration
  PC1   ------> ADC2_INP11
  PB0   ------> ADC2_INN5
  PB1   ------> ADC2_INP5
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* ADC2 DMA Init */

  /* ADC2 Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_1, LL_DMAMUX1_REQ_ADC2);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_1, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_1);

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_DISABLE);
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_16B;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC2, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS;
  ADC_REG_InitStruct.SequencerDiscont = DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC2);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC2);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  __IO uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_5);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_387CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_5, LL_ADC_DIFFERENTIAL_ENDED);
  /* USER CODE BEGIN ADC2_Init 2 */
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_11);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_11, LL_ADC_SAMPLINGTIME_16CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_11, LL_ADC_SINGLE_ENDED);

  ADC2->PCSEL = (1U << 5) | (1U << 11);
  /* USER CODE END ADC2_Init 2 */

}

/* =========================
   TIM/DMA/GPIO (bez zmian)
   ========================= */
static void MX_TIM1_Init(void)
{
    LL_TIM_InitTypeDef      TIM_InitStruct    = {0};
    LL_TIM_OC_InitTypeDef   TIM_OC_InitStruct = {0};
    LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct= {0};
    LL_GPIO_InitTypeDef     GPIO_InitStruct   = {0};

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    /* ===============================
     * TIM1 BASE: 100 kHz PWM
     * =============================== */
    TIM_InitStruct.Prescaler         = 0;
    TIM_InitStruct.CounterMode       = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload        = 2399;          // 100 kHz
    TIM_InitStruct.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(TIM1, &TIM_InitStruct);

    LL_TIM_DisableARRPreload(TIM1);
    LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);

    /* ===============================
     * PWM CH1 + CH1N
     * =============================== */
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);

    TIM_OC_InitStruct.OCMode       = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.OCState      = LL_TIM_OCSTATE_ENABLE;
    TIM_OC_InitStruct.OCNState     = LL_TIM_OCSTATE_ENABLE;
    TIM_OC_InitStruct.CompareValue = 0;               // start = 0%
    TIM_OC_InitStruct.OCPolarity   = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCNPolarity  = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCIdleState  = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;

    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);

    /* ===============================
     * DEAD-TIME + OUTPUT ENABLE
     * =============================== */
    TIM_BDTRInitStruct.OSSRState       = LL_TIM_OSSR_DISABLE;
    TIM_BDTRInitStruct.OSSIState       = LL_TIM_OSSI_DISABLE;
    TIM_BDTRInitStruct.LockLevel       = LL_TIM_LOCKLEVEL_OFF;
    TIM_BDTRInitStruct.DeadTime        = 12;
    TIM_BDTRInitStruct.BreakState      = LL_TIM_BREAK_DISABLE;
    TIM_BDTRInitStruct.BreakPolarity   = LL_TIM_BREAK_POLARITY_HIGH;
    TIM_BDTRInitStruct.BreakFilter     = LL_TIM_BREAK_FILTER_FDIV1;
    TIM_BDTRInitStruct.Break2State     = LL_TIM_BREAK2_DISABLE;
    TIM_BDTRInitStruct.Break2Polarity  = LL_TIM_BREAK_POLARITY_HIGH;
    TIM_BDTRInitStruct.Break2Filter    = LL_TIM_BREAK2_FILTER_FDIV1;
    TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_ENABLE;

    LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);

    /* ===============================
     * GPIO: PE8 (CH1), PE9 (CH1N)
     * =============================== */
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);

    GPIO_InitStruct.Pin        = LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* ===============================
     * START PWM (bez IRQ!)
     * =============================== */
    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_EnableCounter(TIM1);
}


static void MX_TIM2_Init(void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    /* ===============================
     * TIM2 = 10 kHz CONTROL ISR
     * =============================== */
    TIM_InitStruct.Prescaler     = 239;   // 240 MHz / (239+1) = 1 MHz teraz zamiana na 200 Mhz
    TIM_InitStruct.CounterMode   = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload    = 99;    // 1 MHz / 100 = 10 kHz
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;

    LL_TIM_Init(TIM2, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM2);
    LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);

    /* ===============================
     * UPDATE INTERRUPT
     * =============================== */
    LL_TIM_ClearFlag_UPDATE(TIM2);
    LL_TIM_EnableIT_UPDATE(TIM2);

    /* ===============================
     * NVIC
     * =============================== */
    NVIC_SetPriority(TIM2_IRQn, 0);   // niższy priorytet niż ADC/PWM
    NVIC_EnableIRQ(TIM2_IRQn);

    /* ===============================
     * START TIMER
     * =============================== */
    LL_TIM_EnableCounter(TIM2);
}


static void MX_TIM4_Init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(TIM4_IRQn);

  TIM_InitStruct.Prescaler     = 0;
  TIM_InitStruct.CounterMode   = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload    = 1199;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM4);
  LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM4);
}

static void MX_DMA_Init(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  NVIC_SetPriority(DMA1_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
  NVIC_EnableIRQ(DMA1_Stream0_IRQn);

  NVIC_SetPriority(DMA1_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
  NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOF);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);

  GPIO_InitStruct.Pin  = LL_GPIO_PIN_2 | LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin        = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* === FDCAN2 GPIO PB12 RX / PB13 TX === */
  LL_GPIO_InitTypeDef gpio_can = {0};

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);

  gpio_can.Pin = LL_GPIO_PIN_12 | LL_GPIO_PIN_13;
  gpio_can.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_can.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_can.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_can.Pull = LL_GPIO_PULL_NO;
  gpio_can.Alternate = LL_GPIO_AF_9;   // AF9 = FDCAN2
  LL_GPIO_Init(GPIOB, &gpio_can);


//  // === SWO (PB3) ===
//  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
//  // w MX_GPIO_Init() LUB wcześniej
//  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
//  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_3, LL_GPIO_PULL_NO);
//  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_HIGH);
//  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_3, LL_GPIO_AF_0);
//  GPIO_InitStruct.Pin       = LL_GPIO_PIN_3;
//  GPIO_InitStruct.Mode      = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed     = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull      = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;   // SWO
//  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* ========================================================================== */

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

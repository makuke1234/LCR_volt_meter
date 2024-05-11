#include "DAC.hpp"
#include "main.hpp"
#include "samd21/include/component/dmac.h"
#include "samd21/include/samd21g18a.h"

static clk::tmr s_tmr;
static Tcc * s_tcc = nullptr;
static Tc * s_tc = nullptr;
static std::uint32_t s_dmaTrigger = 0;

static const std::uint16_t * s_buf = nullptr;
static std::uint32_t s_bufsize = 0;
static std::uint32_t s_frequency = 0;
static IRQn_Type s_irqn = TCC1_IRQn;

typedef struct
{
  std::uint16_t btctrl;
  std::uint16_t btcnt;
  std::uint32_t srcaddr;
  std::uint32_t dstaddr;
  std::uint32_t descaddr;

} dmacdescriptor;
volatile dmacdescriptor wrb[12] __attribute__ ((aligned (16)));
dmacdescriptor descriptor_section[12] __attribute__ ((aligned (16)));
dmacdescriptor descriptor __attribute__ ((aligned (16)));

#define DMA_CHANNEL 0

static void dma_init()
{
	// trigger on TCC OVF, update TCC duty, circular
  std::uint32_t temp_CHCTRLB_reg;
  
  //SERIAL.println("dma1");
  //delay(50);

  // probably on by default
  PM->AHBMASK.reg |= PM_AHBMASK_DMAC;
  PM->APBBMASK.reg |= PM_APBBMASK_DMAC;
  //SERIAL.println("dma2");
  //delay(50);

  //SERIAL.println("dma22");
  DMAC->BASEADDR.reg = (std::uint32_t)descriptor_section;
  //SERIAL.println("dma3");
  //delay(50);
  DMAC->WRBADDR.reg = (std::uint32_t)wrb;
  //SERIAL.println("dma");
  //delay(50);
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);
  //SERIAL.println("dma4");
  //delay(50);

  DMAC->CHID.reg = DMAC_CHID_ID(DMA_CHANNEL);
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
  DMAC->SWTRIGCTRL.reg &= (std::uint32_t)(~(1 << DMA_CHANNEL));
  temp_CHCTRLB_reg = DMAC_CHCTRLB_LVL(0) | DMAC_CHCTRLB_TRIGSRC(s_dmaTrigger) | DMAC_CHCTRLB_TRIGACT_BEAT;
  DMAC->CHCTRLB.reg = temp_CHCTRLB_reg;
  DMAC->CHINTENSET.bit.TCMPL = 1;
  DMAC->CHINTENSET.bit.SUSP = 1;
  DMAC->CHINTENSET.bit.TERR = 1;

  // Enable DAC interrupt
  //DAC->INTENSET.reg = 0x04;
  //SERIAL.println("dma5");
  //delay(50);
  // circular buffer
  descriptor.descaddr = (std::uint32_t)&descriptor_section[DMA_CHANNEL];
  descriptor.srcaddr = (std::uint32_t)s_buf + s_bufsize * sizeof(std::uint16_t);
  descriptor.dstaddr = (std::uint32_t)&DAC->DATA.reg;
  descriptor.btcnt = s_bufsize;
  descriptor.btctrl = DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_VALID | DMAC_BTCTRL_BLOCKACT_INT;
  memcpy(&descriptor_section[DMA_CHANNEL], &descriptor, sizeof(dmacdescriptor));
  //SERIAL.println("dma6");
  //delay(50);

  // start channel
  DMAC->CHID.reg = DMAC_CHID_ID(DMA_CHANNEL);
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
  //SERIAL.println("dma7");
  //delay(50);
}
static void dma_deinit()
{
  // stop channel
  DMAC->CTRL.bit.DMAENABLE = 0;
  DMAC->CHID.reg = DMAC_CHID_ID(DMA_CHANNEL);
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
  SERIAL.println("dma disable");
}

void DMAC_Handler()
{
  // interrupts DMAC_CHINTENCLR_TERR DMAC_CHINTENCLR_TCMPL DMAC_CHINTENCLR_SUSP
  std::uint8_t active_channel;

  // disable irqs ?
  __disable_irq();
  active_channel =  DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk; // get channel number
  DMAC->CHID.reg = DMAC_CHID_ID(active_channel);
  static volatile std::uint32_t dmadone = DMAC->CHINTFLAG.reg;
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL; // clear
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TERR;
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_SUSP;
  __enable_irq();

  // this interrupt comes exactly at peak signal value

  // external ADC conversion request
  lcrlogic::state.updateLogic = 1;
  //SERIAL.println("IRQ!");
}

static void s_setbuf_tcc(const std::uint16_t * samples, std::uint32_t size);
static void s_setbuf_tc(const std::uint16_t * samples, std::uint32_t size);

static float s_enable_tcc(std::uint32_t frequency, bool flag);
static float s_enable_tc(std::uint32_t frequency, bool flag);

void dac::init(clk::tmr timer)
{
  switch (timer)
  {
  case clk::tmr::tTCC0:
    s_tcc = TCC0;
    s_tc = nullptr;
    s_dmaTrigger = TCC0_DMAC_ID_OVF;
    s_irqn = TCC0_IRQn;
    break;
  case clk::tmr::tTCC1:
    s_tcc = TCC1;
    s_tc = nullptr;
    s_dmaTrigger = TCC1_DMAC_ID_OVF;
    s_irqn = TCC1_IRQn;
    break;
  case clk::tmr::tTCC2:
    s_tcc = TCC2;
    s_tc = nullptr;
    s_dmaTrigger = TCC2_DMAC_ID_OVF;
    s_irqn = TCC2_IRQn;
    break;
  case clk::tmr::tTC3:
    s_tcc = nullptr;
    s_tc = TC3;
    s_dmaTrigger = TC3_DMAC_ID_OVF;
    s_irqn = TC3_IRQn;
    break;
  case clk::tmr::tTC4:
    s_tcc = nullptr;
    s_tc = TC4;
    s_dmaTrigger = TC4_DMAC_ID_OVF;
    s_irqn = TC4_IRQn;
    break;
  case clk::tmr::tTC5:
    s_tcc = nullptr;
    s_tc = TC5;
    s_dmaTrigger = TC5_DMAC_ID_OVF;
    s_irqn = TC5_IRQn;
    break;
  default:
    assert(!"Unsupported timer!");
  }

  s_tmr = timer;

  analogWriteResolution(10);
  analogWrite(DAC_OUT, 512);

  /*
  // set DAC clock to 48 MHz
  GCLK->CLKCTRL.reg =
    GCLK_CLKCTRL_ID_DAC |
    GCLK_CLKCTRL_GEN_GCLK0 |
    GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY);
  */
}

static void s_setbuf_tcc(const std::uint16_t * samples, std::uint32_t size)
{
  assert(s_tcc != nullptr);

  const bool isEnabled = s_tcc->CTRLA.bit.ENABLE == 1;

  if (isEnabled)
  {
    s_enable_tcc(s_frequency, false);
  }
  s_buf = samples;
  s_bufsize = (s_buf != nullptr) ? size : 0;
  if (isEnabled)
  {
    s_enable_tcc(s_frequency, true);
  }
}
static void s_setbuf_tc(const std::uint16_t * samples, std::uint32_t size)
{
  assert(s_tc != nullptr);

  const bool isEnabled = s_tc->COUNT16.CTRLA.bit.ENABLE == 1;

  if (isEnabled)
  {
    s_enable_tc(s_frequency, false);
  }
  s_buf = samples;
  s_bufsize = (s_buf != nullptr) ? size : 0;
  if (isEnabled)
  {
    s_enable_tc(s_frequency, true);
  }
}
void dac::setbuf(const std::uint16_t * samples, std::uint32_t size)
{
  if (s_tcc != nullptr)
  {
    s_setbuf_tcc(samples, size);
  }
  else
  {
    s_setbuf_tc(samples, size);
  }
}

static float s_enable_tcc(std::uint32_t frequency, bool flag)
{
  assert(s_tcc != nullptr);
  assert(s_buf != nullptr);
  assert(frequency > 0);
  const auto isr_freq = frequency;
  SERIAL.print("ISR freq: ");
  SERIAL.println(isr_freq);
  delay(50);

  // enable/disable periodic timer on TCC
  if (flag)
  {
    // recalculate frequency as buffers per cycle
    frequency = frequency * s_bufsize;
    // cap the frequency at 2.048 Msps
    frequency = (frequency > DAC_MAX_SAMPLERATE) ? DAC_MAX_SAMPLERATE : frequency;
    
    SERIAL.println("b1");
    //delay(50);
    dma_init();
    SERIAL.println("b1");
    //delay(50);

    uint32_t tmrFreq = 96000000U;
    if (!clk::isInit(s_tmr))
    {
      clk::initTmr(s_tmr, GCLK_CLKCTRL_GEN_GCLK4_Val, tmrFreq);
    }
    else
    {
      tmrFreq = clk::tmrSpeed(s_tmr);
    }

    std::uint32_t prescaler = (tmrFreq / frequency) / 16777215U + 1U;
    // calculate prescaler register
    auto prescaler_reg = 0;
    {
      auto temppre = prescaler;
      while (temppre >>= 1)
      {
        ++prescaler_reg;
      }
    }
    prescaler_reg = (prescaler_reg > 4U) ? ((prescaler_reg - 4U) / 2U + 4U) : prescaler_reg;
    prescaler_reg = (prescaler_reg > 7U) ? 7U : prescaler_reg;
    // calculate real prescaler
    prescaler = 1U << ((prescaler_reg > 4U) ? (prescaler_reg - 4U) * 2U + 4U : prescaler_reg);
    
    SERIAL.println("b1");
    //delay(50);
    // set prescaler register
    s_tcc->CTRLA.bit.PRESCALER = prescaler_reg;
    SERIAL.println("b1");
    //delay(50);

    // Set up TCC period register according to frequency
    std::uint32_t period = tmrFreq / (prescaler * frequency) - 1;
    // TCC is 24-bit
    period = (period > 16777215U) ? 16777215U : period;

    SERIAL.println("b1");
    //delay(50);
    s_tcc->PER.reg = std::uint16_t(period);
    while (s_tcc->SYNCBUSY.bit.PER);
    SERIAL.println("b1");
    //delay(50);

    /*s_tcc->CC[0].reg = std::uint16_t(period / 2);
    while (s_tcc->SYNCBUSY.bit.CC0);*/

    s_frequency = frequency;

    /*if (isr_freq < 2000)
    {
      NVIC_EnableIRQ(DMAC_IRQn);
      SERIAL.println("Enable DAC IRQ");
    }*/

    SERIAL.println("b1");
    //delay(50);
    // Enable TCC
    s_tcc->CTRLA.bit.ENABLE = 1;
    while (s_tcc->SYNCBUSY.bit.ENABLE);
    SERIAL.println("b1");
    //delay(50);

    // calculate real frequency
    float freq = (float(tmrFreq) / float(prescaler)) / (float(period * s_bufsize));
    return freq;
  }
  else if (s_tcc->CTRLA.bit.ENABLE)
  {
    // Disable TCC
    SERIAL.println("disabling! tcc");
    dma_deinit();

    NVIC_DisableIRQ(DMAC_IRQn);
    
    s_tcc->CTRLA.bit.ENABLE = 0;
    while (s_tcc->SYNCBUSY.bit.ENABLE);
    return 0.0f;
  }
}
static float s_enable_tc(std::uint32_t frequency, bool flag)
{
  assert(s_tc != nullptr);
  assert(s_buf != nullptr);
  assert(frequency > 0);
  const auto isr_freq = frequency;
  SERIAL.print("ISR freq: ");
  SERIAL.println(isr_freq);
  delay(50);

  // enable/disable periodic timer on TC
  if (flag)
  {
    // recalculate frequency as buffers per cycle
    frequency = frequency * s_bufsize;
    // cap the frequency at 2.048 Msps
    frequency = (frequency > DAC_MAX_SAMPLERATE) ? DAC_MAX_SAMPLERATE : frequency;
    
    SERIAL.println("b1");
    //delay(50);
    dma_init();
    SERIAL.println("b1");
    //delay(50);

    uint32_t tmrFreq = 96000000U;
    if (!clk::isInit(s_tmr))
    {
      clk::initTmr(s_tmr, GCLK_CLKCTRL_GEN_GCLK4_Val, tmrFreq);
    }
    else
    {
      tmrFreq = clk::tmrSpeed(s_tmr);
    }

    std::uint32_t prescaler = (tmrFreq / frequency) / 65535U + 1U;
    // calculate prescaler register
    auto prescaler_reg = 0;
    {
      auto temppre = prescaler;
      while (temppre >>= 1)
      {
        ++prescaler_reg;
      }
    }
    prescaler_reg = (prescaler_reg > 4U) ? ((prescaler_reg - 4U) / 2U + 4U) : prescaler_reg;
    prescaler_reg = (prescaler_reg > 7U) ? 7U : prescaler_reg;
    // calculate real prescaler
    prescaler = 1U << ((prescaler_reg > 4U) ? (prescaler_reg - 4U) * 2U + 4U : prescaler_reg);
    
    // set prescaler register
    SERIAL.println("b1");
    delay(50);
    s_tc->COUNT16.CTRLA.bit.PRESCALER = prescaler_reg;
    s_tc->COUNT16.CTRLA.reg |=
      TC_CTRLA_MODE_COUNT16 |
      TC_CTRLA_WAVEGEN_MFRQ;
    while (s_tc->COUNT16.STATUS.bit.SYNCBUSY);
    SERIAL.println("b1");
    //delay(50);

    // Set up TC period register according to frequency
    std::uint32_t period = tmrFreq / (prescaler * frequency) - 1;
    // TC is 16-bit
    period = (period > 65535U) ? 65535U : period;

    SERIAL.println("b1");
    //delay(50);
    s_tc->COUNT16.CC[0].reg = std::uint16_t(period);
    while (s_tc->COUNT16.STATUS.bit.SYNCBUSY);
    SERIAL.println("b1");
    //delay(50);

    s_frequency = frequency;

    /*if (isr_freq < 2000)
    {
      NVIC_EnableIRQ(DMAC_IRQn);
      SERIAL.println("Enable DAC IRQ");
    }*/

    // Enable TC
    SERIAL.println("b1");
    //delay(50);
    s_tc->COUNT16.CTRLA.bit.ENABLE = 1;
    while (s_tc->COUNT16.STATUS.bit.SYNCBUSY);
    SERIAL.println("b1");
    //delay(50);

    float freq = (float(tmrFreq) / float(prescaler)) / (float(period * s_bufsize));
    return freq;
  }
  else
  {
    // Disable TC
    SERIAL.println("disabling! tc");
    dma_deinit();

    NVIC_DisableIRQ(DMAC_IRQn);

    s_tc->COUNT16.CTRLA.bit.ENABLE = 0;
    while (s_tc->COUNT16.STATUS.bit.SYNCBUSY);
    return 0.0f;
  }
}
float dac::enable(std::uint32_t frequency, bool flag)
{
  if ((s_buf == nullptr) || (s_bufsize == 0))
  {
    return 0.0f;
  }

  if (s_tcc != nullptr)
  {
    return s_enable_tcc(frequency, flag);
  }
  else
  {
    return s_enable_tc(frequency, flag);
  }
}


#include "Arduino.h"

// Code arduino DUE - emission sinusoidale a 40kHz, ampli PP 3.3V
//Bug dans l'init -> mauvaise frequence

#define SET_SYNTHESE_TABLE 101
#define SET_FREQUENCE 102
#define NECHANT 128
#define SHIFT_ACCUM 25
uint16_t table_0[NECHANT];
uint16_t table_1[NECHANT];
uint8_t channel;
uint32_t chsel;
uint32_t accum_0, accum_1;
uint32_t increm;
uint32_t ticks;
uint8_t timer_actif;
volatile void (*TC0_function)();
             
void declencher_timer(uint32_t ticks, volatile void (*function)()) {
   uint8_t clock = TC_CMR_TCCLKS_TIMER_CLOCK1; // horloge 84MHz/2=42 MHz
   uint32_t channel = 0;
   TC0_function = function;
   pmc_set_writeprotect(false);
   pmc_enable_periph_clk((uint32_t)TC0_IRQn);
   TC_Configure(TC0, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | clock);
   TC0->TC_CHANNEL[channel].TC_RC = ticks;
   TC_Start(TC0, channel);
   TC0->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
   TC0->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
   timer_actif = 1;
   NVIC_EnableIRQ(TC0_IRQn);
}
              
void stopper_timer() {
   NVIC_DisableIRQ(TC0_IRQn);
   pmc_disable_periph_clk((uint32_t)TC0_IRQn); 
   timer_actif = 0;
}
               
void TC0_Handler()
{
   TC_GetStatus(TC0, 0);
   (*TC0_function)();
}
               
void configurer_dac(uint32_t channel) {
   pmc_enable_periph_clk(DACC_INTERFACE_ID);
   dacc_reset(DACC_INTERFACE);
   dacc_set_transfer_mode(DACC_INTERFACE, 0); // half word transfert (16 bits)
   dacc_set_power_save(DACC_INTERFACE, 0, 0);
   dacc_set_timing(DACC_INTERFACE, 0x08, 0, 0x10);
   dacc_set_analog_control(DACC_INTERFACE, DACC_ACR_IBCTLCH0(0x02)|DACC_ACR_IBCTLCH1(0x02)|DACC_ACR_IBCTLDACCORE(0x01));
   dacc_set_channel_selection(DACC_INTERFACE, channel);
   if ((dacc_get_channel_status(DACC_INTERFACE) & (1 << channel)) == 0) 
				dacc_enable_channel(DACC_INTERFACE, channel);
}     
               
void configurer_double_dac() {
   pmc_enable_periph_clk(DACC_INTERFACE_ID);
   dacc_reset(DACC_INTERFACE);
   dacc_set_transfer_mode(DACC_INTERFACE, 1); //  word transfert (32 bits)
   dacc_set_power_save(DACC_INTERFACE, 0, 0);
   dacc_set_timing(DACC_INTERFACE, 0x08, 0, 0x10);
   dacc_set_analog_control(DACC_INTERFACE, DACC_ACR_IBCTLCH0(0x02)|DACC_ACR_IBCTLCH1(0x02)|DACC_ACR_IBCTLDACCORE(0x01));
   DACC_INTERFACE->DACC_MR |= DACC_MR_TAG;
   DACC_INTERFACE->DACC_CHER = 0x3;
   chsel = (1<<13) | (1<<28);
}
               
volatile void synthese_table() {
    accum_0 += increm;
    DACC_INTERFACE->DACC_CDR = table_0[accum_0 >> SHIFT_ACCUM];
}

volatile void synthese_table_double() {
    accum_0 += increm;
    accum_1 += increm;
    DACC_INTERFACE->DACC_CDR = table_0[accum_0 >> SHIFT_ACCUM] | (table_1[accum_1 >> SHIFT_ACCUM] << 16) | chsel; 
}
               
void setup() {
  SerialUSB.begin(115200);
}
               
void lecture_synthese_table() {
    char com;
    uint32_t c1,c2,c3,c4;
    char k;
    uint32_t frequence;
    stopper_timer();
    while (SerialUSB.available()<1) {};
    channel = SerialUSB.read(); // voie : 0,1 ou 2
    while (SerialUSB.available()<4) {};
    c1 = SerialUSB.read();
    c2 = SerialUSB.read();
    c3 = SerialUSB.read();
    c4 = SerialUSB.read();
    ticks = ((c1 << 24) | (c2 << 16) | (c3 << 8) | c4); // nombre de tops d'horloge (42 MHz) pour la période d'échantillonnage
    while (SerialUSB.available()<4) {};
    c1 = SerialUSB.read();
    c2 = SerialUSB.read();
    c3 = SerialUSB.read();
    c4 = SerialUSB.read();
    frequence = ((c1 << 24) | (c2 << 16) | (c3 << 8) | c4); // fréquence en millième de Hz
    float fechant = 42.0e6/ticks;
    increm = (uint32_t) (((float)(0xFFFFFFFF))*((float)(frequence)*0.001/fechant)); // incrément de l'accumulateur de phase
    for (k=0; k<NECHANT; k++) {
        while (SerialUSB.available()<2) {};
        c1 = SerialUSB.read();
        c2 = SerialUSB.read();
        table_0[k] =  ((c1<<8)|c2);
    }
    if (channel==2) {
      for (k=0; k<NECHANT; k++) {
        while (SerialUSB.available()<2) {};
        c1 = SerialUSB.read();
        c2 = SerialUSB.read();
        table_1[k] =  ((c1<<8)|c2);
      }     
    }
    accum_0 = 0;
    accum_1 = 0;
    if (channel==2) {
      configurer_double_dac();
      declencher_timer(ticks,synthese_table_double);
    }
    else {
      configurer_dac(channel);
      declencher_timer(ticks,synthese_table);
    }
} 
               
void lecture_frequence() {
    uint32_t c1,c2,c3,c4;
    uint32_t frequence;
    while (SerialUSB.available()<4) {};
    c1 = SerialUSB.read();
    c2 = SerialUSB.read();
    c3 = SerialUSB.read();
    c4 = SerialUSB.read();
    frequence = ((c1 << 24) | (c2 << 16) | (c3 << 8) | c4); // fréquence en millième de Hz
    float fechant = 42.0e6/ticks;
    increm = (uint32_t) (((float)(0xFFFFFFFF))*((float)(frequence)*0.001/fechant));   
}
              
void lecture_serie() {
  char com;
  if (SerialUSB.available()>0) {
        com = SerialUSB.read();
        if (com==SET_SYNTHESE_TABLE) lecture_synthese_table();
        else if (com==SET_FREQUENCE) lecture_frequence();
  }
}
               
void loop() {
   lecture_serie();
}
               

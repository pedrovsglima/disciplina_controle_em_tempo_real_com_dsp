/*
 * Pedro_Yago.c
 *
 *  Created on: 29/01/2018
 *      Author: UFPE
 */
#include "DSP2833x_Device.h"

//#define Ki    0.02538	//ajustar
//#define Kp    1.933	//ajustar
#define Ki    0.02738	//ajustar
#define Kp    2.675	//ajustar
#define Ts	  1	//s
#define P_max 69	//ajustar potencia maxima
float conv_tensao=0.00073260073260073260073260073260073;  //evitar a div por 3/4095
float conv_temperatura=100; //LM35
#define ganho 4 //ganho amplificador

float T_real;
float T_ref = 40;
float V_ref = 0;
float conv_amplificador=1/ganho;
float pu_potencia=1/P_max;

float Voltage_Sensor;
float error = 0;
float prev_error = 0;
float saida = 0;
float duty = 0;

// external function prototypes
extern void InitAdc(void);
extern void InitSysCtrl(void);
extern void InitPieCtrl(void);
extern void InitPieVectTable(void);
extern void InitCpuTimers(void);
extern void ConfigCpuTimer(struct CPUTIMER_VARS *, float, float);
extern void display_ADC(unsigned int);

// Prototype statements for functions found within this file.
void Controle(void);
void Gpio_select(void);
void Setup_ePWM1A(void);
interrupt void cpu_timer0_isr(void);
interrupt void adc_isr(void);		 // ADC  End of Sequence ISR



//###########################################################################
//						Código principal
//###########################################################################
void main(void)
{
	InitSysCtrl();	// Basic Core Init from DSP2833x_SysCtrl.c

	EALLOW;
	SysCtrlRegs.WDCR = 0x00AF;	// Re-enable the watchdog
	EDIS;			// 0x00AF  to NOT disable the Watchdog, Prescaler = 64

	DINT;				// Disable all interrupts

	Gpio_select();		// GPIO9, GPIO11, GPIO34 and GPIO49 as output
						// to 4 LEDs at Peripheral Explorer)

	Setup_ePWM1A();

	InitPieCtrl();		// basic setup of PIE table; from DSP2833x_PieCtrl.c

	InitPieVectTable();	// default ISR's in PIE

	InitAdc();			// Basic ADC setup, incl. calibration

	AdcRegs.ADCTRL1.all = 0;
	AdcRegs.ADCTRL1.bit.ACQ_PS = 7; 	// 7 = 8 x ADCCLK
	AdcRegs.ADCTRL1.bit.SEQ_CASC = 1; 	// 1=cascaded sequencer
	AdcRegs.ADCTRL1.bit.CPS = 0;		// divide by 1
	AdcRegs.ADCTRL1.bit.CONT_RUN = 0;	// single run mode

	AdcRegs.ADCTRL2.all = 0;
	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;	// 1=enable SEQ1 interrupt
	AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;	// 1=SEQ1 start from ePWM_SOCA trigger
	AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 0;	// 0= interrupt after every end of sequence

	AdcRegs.ADCTRL3.bit.ADCCLKPS = 3;

	AdcRegs.ADCMAXCONV.all = 0x0001;    // 2 conversions from Sequencer 1

	AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0; // Setup ADCINA0 as 1st SEQ1 conv.
	AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 1; // Setup ADCINA1 as 2nd SEQ1 conv.

	EALLOW;
	PieVectTable.TINT0 = &cpu_timer0_isr;
	PieVectTable.ADCINT = &adc_isr;
	EDIS;

	InitCpuTimers();	// basic setup CPU Timer0, 1 and 2

	ConfigCpuTimer(&CpuTimer0, 150, 100000);

	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;		// CPU Timer 0
	PieCtrlRegs.PIEIER1.bit.INTx6 = 1;		// ADC

	IER |= 1;

	EINT;
	ERTM;

	CpuTimer0Regs.TCR.bit.TSS = 0;	// start timer0

	while (1)
	{
			EALLOW;
			SysCtrlRegs.WDKEY = 0x55;		// Service watchdog #1
			EDIS;
	}
}

void Gpio_select(void)
{
	EALLOW;
	GpioCtrlRegs.GPAMUX1.all = 0;		// GPIO15 ... GPIO0 = General Puropse I/O
	GpioCtrlRegs.GPAMUX2.all = 0;		// GPIO31 ... GPIO16 = General Purpose I/O
	GpioCtrlRegs.GPBMUX1.all = 0;		// GPIO47 ... GPIO32 = General Purpose I/O
	GpioCtrlRegs.GPBMUX2.all = 0;		// GPIO63 ... GPIO48 = General Purpose I/O
	GpioCtrlRegs.GPCMUX1.all = 0;		// GPIO79 ... GPIO64 = General Purpose I/O
	GpioCtrlRegs.GPCMUX2.all = 0;		// GPIO87 ... GPIO80 = General Purpose I/O

	GpioCtrlRegs.GPADIR.all = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 01;

	GpioCtrlRegs.GPBDIR.all = 0;		// GPIO63-32 as inputs
	GpioCtrlRegs.GPCDIR.all = 0;		// GPIO87-64 as inputs
	EDIS;
}

interrupt void cpu_timer0_isr(void)
{
	CpuTimer0.InterruptCount++;
	EALLOW;
	SysCtrlRegs.WDKEY = 0xAA;	// service WD #2
	EDIS;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void  adc_isr(void)
{
	Voltage_Sensor = AdcMirror.ADCRESULT0*conv_tensao;	// store results global

	//T_real = Voltage_Sensor*conv_amplificador*conv_temperatura;
	T_real = Voltage_Sensor*25;

	Controle();

	// Reinitialize for next ADC sequence
	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;       // Reset SEQ1
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;		// Clear INT SEQ1 bit
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge interrupt to PIE
}

void Setup_ePWM1A(void)
{
	EPwm1Regs.CMPA.half.CMPA = 0; // largura de pulso de 100%
	EPwm1Regs.TBCTL.all = 0x1234;		//default
	EPwm1Regs.TBCTL.bit.CLKDIV = 7;	// CLKDIV = 128 = 0b111
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 6;	// HSPCLKDIV = 12 = 0b110
	EPwm1Regs.TBCTL.bit.CTRMODE = 2;	// up - down mode
	EPwm1Regs.TBPRD = 48828; //formula da pagina 11 com freq de 1Hz
	EPwm1Regs.AQCTLA.all = 0x0060; // onda quadrada
	EPwm1Regs.AQCTLB.all = 0x0090;
	EPwm1Regs.ETPS.all=0;
	EPwm1Regs.ETPS.bit.SOCAPRD=1;
	EPwm1Regs.ETSEL.bit.SOCAEN=1;
	EPwm1Regs.ETSEL.bit.SOCASEL=2;
}

void Controle(void) {

	error = T_ref - T_real;

	//saida = error*Kp + Ki*(saida + Ts*0.5*(error + prev_error));  //PI discretizado por Tustin
	saida = saida + error*(Kp + Ki*Ts) - Kp*prev_error; //backward

	//duty = (saida*pu_potencia);
	duty = saida * 0.01449275362318840579710144927536;

	if (duty<= 0) { duty = 0; }

	EPwm1Regs.CMPA.half.CMPA = EPwm1Regs.TBPRD*duty;

	prev_error = error;
}

//===========================================================================
// Fim do código
//===========================================================================

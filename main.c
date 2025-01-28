/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "uart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {false, true} bool;
typedef enum {__Menu, __LED, __LED7, __Joy, __LPUART} Level;
typedef enum {
	HELP,
	LED,
	LED7,
	Joy,
	LPUART,
	UP,
	Set,
	Clear,
	Blink,
	Toggle,
	Display,
	Read,
	Status,
	Invalid
	} Command;

typedef struct
{
  volatile uint32_t MODER;       /*!< GPIO port mode register,               Address offset: 0x00      */
  volatile uint32_t OTYPER;      /*!< GPIO port output type register,        Address offset: 0x04      */
  volatile uint32_t OSPEEDR;     /*!< GPIO port output speed register,       Address offset: 0x08      */
  volatile uint32_t PUPDR;       /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  volatile uint32_t IDR;         /*!< GPIO port input data register,         Address offset: 0x10      */
  volatile uint32_t ODR;         /*!< GPIO port output data register,        Address offset: 0x14      */
  volatile uint32_t BSRR;        /*!< GPIO port bit set/reset  register,     Address offset: 0x18      */
  volatile uint32_t LCKR;        /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  volatile uint32_t AFR[2];      /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
  volatile uint32_t BRR;         /*!< GPIO Bit Reset register,               Address offset: 0x28      */

} Perph;

typedef struct
{
	Perph *perph;
	uint16_t number;
} Led;

typedef struct
{
	volatile uint32_t CR;          /*!< RCC clock control register,                                              Address offset: 0x00 */
	volatile uint32_t ICSCR;       /*!< RCC internal clock sources calibration register,                         Address offset: 0x04 */
	volatile uint32_t CFGR;        /*!< RCC clock configuration register,                                        Address offset: 0x08 */
	volatile uint32_t PLLCFGR;     /*!< RCC system PLL configuration register,                                   Address offset: 0x0C */
	volatile uint32_t PLLSAI1CFGR; /*!< RCC PLL SAI1 configuration register,                                     Address offset: 0x10 */
	volatile uint32_t PLLSAI2CFGR; /*!< RCC PLL SAI2 configuration register,                                     Address offset: 0x14 */
	volatile uint32_t CIER;        /*!< RCC clock interrupt enable register,                                     Address offset: 0x18 */
	volatile uint32_t CIFR;        /*!< RCC clock interrupt flag register,                                       Address offset: 0x1C */
	volatile uint32_t CICR;        /*!< RCC clock interrupt clear register,                                      Address offset: 0x20 */
	uint32_t      RESERVED0;   /*!< Reserved,                                                                Address offset: 0x24 */
	volatile uint32_t AHB1RSTR;    /*!< RCC AHB1 peripheral reset register,                                      Address offset: 0x28 */
	volatile uint32_t AHB2RSTR;    /*!< RCC AHB2 peripheral reset register,                                      Address offset: 0x2C */
	volatile uint32_t AHB3RSTR;    /*!< RCC AHB3 peripheral reset register,                                      Address offset: 0x30 */
	uint32_t      RESERVED1;   /*!< Reserved,                                                                Address offset: 0x34 */
	volatile uint32_t APB1RSTR1;   /*!< RCC APB1 peripheral reset register 1,                                    Address offset: 0x38 */
	volatile uint32_t APB1RSTR2;   /*!< RCC APB1 peripheral reset register 2,                                    Address offset: 0x3C */
	volatile uint32_t APB2RSTR;    /*!< RCC APB2 peripheral reset register,                                      Address offset: 0x40 */
	uint32_t      RESERVED2;   /*!< Reserved,                                                                Address offset: 0x44 */
	volatile uint32_t AHB1ENR;     /*!< RCC AHB1 peripheral clocks enable register,                              Address offset: 0x48 */
	volatile uint32_t AHB2ENR;     /*!< RCC AHB2 peripheral clocks enable register,                              Address offset: 0x4C */
	volatile uint32_t AHB3ENR;     /*!< RCC AHB3 peripheral clocks enable register,                              Address offset: 0x50 */
	uint32_t      RESERVED3;   /*!< Reserved,                                                                Address offset: 0x54 */
	volatile uint32_t APB1ENR1;    /*!< RCC APB1 peripheral clocks enable register 1,                            Address offset: 0x58 */
	volatile uint32_t APB1ENR2;    /*!< RCC APB1 peripheral clocks enable register 2,                            Address offset: 0x5C */
	volatile uint32_t APB2ENR;     /*!< RCC APB2 peripheral clocks enable register,                              Address offset: 0x60 */
	uint32_t      RESERVED4;   /*!< Reserved,                                                                Address offset: 0x64 */
	volatile uint32_t AHB1SMENR;   /*!< RCC AHB1 peripheral clocks enable in sleep and stop modes register,      Address offset: 0x68 */
	volatile uint32_t AHB2SMENR;   /*!< RCC AHB2 peripheral clocks enable in sleep and stop modes register,      Address offset: 0x6C */
	volatile uint32_t AHB3SMENR;   /*!< RCC AHB3 peripheral clocks enable in sleep and stop modes register,      Address offset: 0x70 */
	uint32_t      RESERVED5;   /*!< Reserved,                                                                Address offset: 0x74 */
	volatile uint32_t APB1SMENR1;  /*!< RCC APB1 peripheral clocks enable in sleep mode and stop modes register 1, Address offset: 0x78 */
	volatile uint32_t APB1SMENR2;  /*!< RCC APB1 peripheral clocks enable in sleep mode and stop modes register 2, Address offset: 0x7C */
	volatile uint32_t APB2SMENR;   /*!< RCC APB2 peripheral clocks enable in sleep mode and stop modes register, Address offset: 0x80 */
	uint32_t      RESERVED6;   /*!< Reserved,                                                                Address offset: 0x84 */
	volatile uint32_t CCIPR;       /*!< RCC peripherals independent clock configuration register,                Address offset: 0x88 */
    uint32_t      RESERVED7;   /*!< Reserved,                                                                Address offset: 0x8C */
	volatile uint32_t BDCR;        /*!< RCC backup domain control register,                                      Address offset: 0x90 */
	volatile uint32_t CSR;         /*!< RCC clock control & status register,                                     Address offset: 0x94 */
	volatile uint32_t CRRCR;       /*!< RCC clock recovery RC register,                                          Address offset: 0x98 */
	volatile uint32_t CCIPR2;      /*!< RCC peripherals independent clock configuration register 2,              Address offset: 0x9C */

} Clock;

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMCR;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR1;
	volatile uint32_t CCMR2;
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
	uint32_t RESERVED0;
	volatile uint32_t CCR1;
	volatile uint32_t CCR2;
	volatile uint32_t CCR3;
	volatile uint32_t CCR4;
	uint32_t RESERVED1;
	volatile uint32_t DCR;
	volatile uint32_t DMAR;
	volatile uint32_t OR1;
	uint32_t RESERVED2;
	uint32_t RESERVED3;
	uint32_t RESERVED4;
	volatile uint32_t OR2;
} GPTim2_3;

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t CR4;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t SCR;
	volatile uint32_t PUCRA;
	volatile uint32_t PDCRA;
	volatile uint32_t PUCRB;
	volatile uint32_t PDCRB;
	volatile uint32_t PUCRC;
	volatile uint32_t PDCRC;
	volatile uint32_t PUCRD;
	volatile uint32_t PDCRD;
	volatile uint32_t PUCRE;
	volatile uint32_t PDCRE;
	volatile uint32_t PUCRF;
	volatile uint32_t PDCRF;
	volatile uint32_t PUCRG;
	volatile uint32_t PDCRG;
	volatile uint32_t PUCRH;
	volatile uint32_t PDCRH;
	volatile uint32_t PUCRI;
	volatile uint32_t PDCRI;
} Power;

typedef struct
{
	volatile uint32_t ACR;
	volatile uint32_t PDKEYR;
	volatile uint32_t KEYR;
	volatile uint32_t OPTKEYR;
	volatile uint32_t SR;
	volatile uint32_t CR;
	volatile uint32_t ECCR;
	uint32_t RESERVED0;
	volatile uint32_t OPTR;
	volatile uint32_t PCROP1SR;
	volatile uint32_t PCROP1ER;
	volatile uint32_t WRP1AR;
	volatile uint32_t WRP1BR;
	uint32_t RESERVED1;
	uint32_t RESERVED2;
	uint32_t RESERVED3;
	uint32_t RESERVED4;
	volatile uint32_t PCROP2SR;
	volatile uint32_t PCROP2ER;
	volatile uint32_t WRP2AR;
	volatile uint32_t WRP2BR;
} Flash;

typedef struct
{
	uint32_t Pin;
	uint32_t Mode;
	uint32_t Speed;
	uint32_t Pull;
	uint32_t Alternate;
} Pin_InitStruct;

typedef struct
{
	uint32_t OscType;
	uint32_t Range;
	uint32_t State;
} Osc_InitStruct;

typedef struct
{
	uint32_t SYSCLKSource;
	uint32_t HCLK_Div;
	uint32_t PCLK1_Div;
} Clk_InitStruct;

typedef struct
{
	uint32_t Dir;
	uint32_t Mode;
	uint32_t State;
	uint32_t Prescaler;
	uint32_t Reload;
} Tim_InitStruct;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define speed 1

#define APB1_BASE 0x40000000UL
#define AHB1_BASE (APB1_BASE + 0x00020000UL)
#define AHB2_BASE (APB1_BASE + 0x08000000UL)

#define GPIOA_BASE 	(AHB2_BASE + 0x0000UL)
#define GPIOB_BASE 	(AHB2_BASE + 0x0400UL)
#define GPIOC_BASE 	(AHB2_BASE + 0x0800UL)
#define GPIOD_BASE 	(AHB2_BASE + 0x0C00UL)
#define GPIOE_BASE 	(AHB2_BASE + 0x1000UL)
#define GPIOF_BASE 	(AHB2_BASE + 0x1400UL)
#define GPIOG_BASE 	(AHB2_BASE + 0x1800UL)
#define GPIOH_BASE 	(AHB2_BASE + 0x1C00UL)
#define GPIOI_BASE 	(AHB2_BASE + 0x2000UL)
#define RCC_BASE   	(AHB1_BASE + 0x1000UL)
#define PWR_BASE   	(APB1_BASE + 0x7000UL)
#define FLASH_BASE 	(AHB1_BASE + 0x2000UL)

#define GPIOA 	((Perph*) GPIOA_BASE)
#define GPIOB 	((Perph*) GPIOB_BASE)
#define GPIOC 	((Perph*) GPIOC_BASE)
#define GPIOD 	((Perph*) GPIOD_BASE)
#define GPIOE 	((Perph*) GPIOE_BASE)
#define GPIOF 	((Perph*) GPIOF_BASE)
#define GPIOG 	((Perph*) GPIOG_BASE)
#define GPIOH 	((Perph*) GPIOH_BASE)
#define GPIOI 	((Perph*) GPIOI_BASE)
#define RCC   	((Clock*) RCC_BASE)
#define PWR   	((Power*) PWR_BASE)
#define FLASH 	((Flash*) FLASH_BASE)
#define TIM2  	((GPTim2_3*) APB1_BASE)

#define GPIO_MODE_INPUT     ((uint32_t) 0x00)
#define GPIO_MODE_OUTPUT    ((uint32_t) 0x01)
#define GPIO_MODE_ALTER     ((uint32_t) 0x02)
#define GPIO_MODE_ANALOG    ((uint32_t) 0x03)
#define GPIO_MODE_PP        ((uint32_t) 0x00)
#define GPIO_MODE_OD        ((uint32_t) 0x01)
#define GPIO_MODE_OUTPUT_PP ((GPIO_MODE_OUTPUT << 1) | GPIO_MODE_PP)
#define GPIO_MODE_OUTPUT_OD ((GPIO_MODE_OUTPUT << 1) | GPIO_MODE_OD)
#define GPIO_MODE_ALTER_PP ((GPIO_MODE_ALTER << 1) | GPIO_MODE_PP)
#define GPIO_MODE_ALTER_OD ((GPIO_MODE_ALTER << 1) | GPIO_MODE_OD)

#define GPIO_SPEED_LOW    ((uint32_t) 0x00)
#define GPIO_SPEED_MEDIUM ((uint32_t) 0x01)
#define GPIO_SPEED_HIGH   ((uint32_t) 0x02)
#define GPIO_SPEED_VHIGH  ((uint32_t) 0x03)

#define GPIO_PULL_NO     ((uint32_t) 0x00)
#define GPIO_PULL_UP     ((uint32_t) 0x01)
#define GPIO_PULL_DOWN   ((uint32_t) 0x02)

#define OSCILLATORTYPE_MSI   (0x01U)
#define OSCILLATORTYPE_LSI   (0x02U)
#define OSCILLATORTYPE_LSE   (0x04U)
#define OSCILLATORTYPE_HSI   (0x08U)
#define OSCILLATORTYPE_HSI48 (0x10U)
#define OSCILLATORTYPE_HSE   (0x20U)

#define MSI_CLK_RANGE0  (0x0UL << 4)
#define MSI_CLK_RANGE1  (0x1UL << 4)
#define MSI_CLK_RANGE2  (0x2UL << 4)
#define MSI_CLK_RANGE3  (0x3UL << 4)
#define MSI_CLK_RANGE4  (0x4UL << 4)
#define MSI_CLK_RANGE5  (0x5UL << 4)
#define MSI_CLK_RANGE6  (0x6UL << 4)
#define MSI_CLK_RANGE7  (0x7UL << 4)
#define MSI_CLK_RANGE8  (0x8UL << 4)
#define MSI_CLK_RANGE9  (0x9UL << 4)
#define MSI_CLK_RANGE10 (0xAUL << 4)
#define MSI_CLK_RANGE11 (0xBUL << 4)

#define MSI_CLK_RANGE_SEL_CSR (0x0UL << 3)
#define MSI_CLK_RANGE_SEL_CR  (0x1UL << 3)

#define MSI_ON  (0x1UL << 0)
#define MSI_OFF (0x0UL << 0)

#define SYSCLK_SOURCE_MSI (0x0UL << 0)

#define AHB_CLK_PRESCALER1   (0x0UL << 4)
#define AHB_CLK_PRESCALER2   (0x8UL << 4)
#define AHB_CLK_PRESCALER4   (0x9UL << 4)
#define AHB_CLK_PRESCALER8   (0xAUL << 4)
#define AHB_CLK_PRESCALER16  (0xBUL << 4)
#define AHB_CLK_PRESCALER64  (0xCUL << 4)
#define AHB_CLK_PRESCALER128 (0xDUL << 4)
#define AHB_CLK_PRESCALER256 (0xEUL << 4)
#define AHB_CLK_PRESCALER512 (0xFUL << 4)

#define APB1_CLK_PRESCALER1   (0x0UL << 8)
#define APB1_CLK_PRESCALER2   (0x4UL << 8)
#define APB1_CLK_PRESCALER4   (0x5UL << 8)
#define APB1_CLK_PRESCALER8   (0x6UL << 8)
#define APB1_CLK_PRESCALER16  (0x7UL << 8)

#define TIM2_CNT_EAM    (0x0UL << 5)
#define TIM2_CNT_UP     (0x0UL << 4)
#define TIM2_CNT_ON     (0x1UL << 0)
#define TIM2_CNT_PSC    0x3UL
#define TIM2_CNT_RELOAD ((uint32_t) 1000)
#define TIM2_DIER_UIE   (0x1UL << 0)

#define INSTRUCTION_CACHE_ENABLE 0x1U
#define DATA_CACHE_ENABLE		 0x1U
#define PREFETCH_ENABLE			 0x0U

#define LPUART1_SRC_PCLK	(0x0UL << 10)
#define LPUART1_SRC_SYSCLK 	(0x1UL << 10)

#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority, 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority, 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority, 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority, 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority, 0 bit  for subpriority*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define GPIOB_CLK_ENABLE()  		(RCC->AHB2ENR |= (0x1UL << 1))
#define GPIOC_CLK_ENABLE()  		(RCC->AHB2ENR |= (0x1UL << 2))
#define GPIOD_CLK_ENABLE()			(RCC->AHB2ENR |= (0x1UL << 3))
#define GPIOE_CLK_ENABLE()  		(RCC->AHB2ENR |= (0x1UL << 4))
#define GPIOG_CLK_ENABLE()  		(RCC->AHB2ENR |= (0x1UL << 6))
#define TIM2_CLK_ENABLE()   		(RCC->APB1ENR1 |= 0x1UL)
#define SYSCFG_CLK_ENABLE() 		(RCC->APB2ENR |= (0x1UL << 0))
#define PWR_CLK_ENABLE() 			(RCC->APB1ENR1 |= (0x1UL << 28))
#define LPUART1_CLK_ENABLE()		(RCC->APB1ENR2 |= 0x1UL)

#define PWR_VDDIO2_ENABLE() 			  (PWR->CR2 |= (0x1UL << 9))
#define FLASH_INSTRUCTION_CACHE_DISABLE() (FLASH->ACR &= ~(0x1UL << 9))
#define FLASH_DATA_CACHE_DISABLE()		  (FLASH->ACR &= ~(0x1UL << 10))
#define FLASH_PREFETCH_BUFFER_ENABLE()	  (FLASH->ACR |= (0x1UL << 8))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t glTick = 0;

Level level = __Menu;
bool show = false;
bool blink = false;

Perph* perph;
uint16_t pin;

char count[4];

Led leds[11] = {
	{.perph = GPIOC, .number = (1 << 6)}, //LED0
	{.perph = GPIOC, .number = (1 << 7)}, //LED1
	{.perph = GPIOC, .number = (1 << 8)}, //LED2
	{.perph = GPIOC, .number = (1 << 9)}, //LED3
	{.perph = GPIOE, .number = (1 << 4)}, //LED4
	{.perph = GPIOD, .number = (1 << 3)}, //LED5
	{.perph = GPIOE, .number = (1 << 5)}, //LED6
	{.perph = GPIOE, .number = (1 << 6)}, //LED7

	{.perph = GPIOD, .number = (1 << 13)}, //LED R
	{.perph = GPIOB, .number = (1 << 8)},  //LED G
	{.perph = GPIOD, .number = (1 << 12)}  //LED B
};

uint16_t digits[10] = {
	(1 << 0)|(1 << 1)|(1 << 2)|(1 << 3)|(1 << 4)|(1 << 5),			//0
	(1 << 1)|(1 << 2),												//1
	(1 << 0)|(1 << 1)|(1 << 3)|(1 << 4)|(1 << 6),					//2
	(1 << 0)|(1 << 1)|(1 << 2)|(1 << 3)|(1 << 6),					//3
	(1 << 1)|(1 << 2)|(1 << 5)|(1 << 6),							//4
	(1 << 0)|(1 << 2)|(1 << 3)|(1 << 5)|(1 << 6),					//5
	(1 << 0)|(1 << 2)|(1 << 3)|(1 << 4)|(1 << 5)|(1 << 6),			//6
	(1 << 0)|(1 << 1)|(1 << 2),										//7
	(1 << 0)|(1 << 1)|(1 << 2)|(1 << 3)|(1 << 4)|(1 << 5)|(1 << 6),	//8
	(1 << 0)|(1 << 1)|(1 << 2)|(1 << 3)|(1 << 5)|(1 << 6)			//9
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void System_Init();

void Osc_Init(Osc_InitStruct* initstruct);
void Clk_Init(Clk_InitStruct* initstruct);
void Tim_Init(Tim_InitStruct* initstruct);

void Clock_Config(void);

void GPIO_Init(Perph* perph, Pin_InitStruct* initstruct);
void GPIO_Config();
uint32_t GPIO_ReadPin(Perph* perph, uint16_t pin);
void GPIO_WritePin(Perph* perph, uint16_t pin, uint16_t pinstate);
void GPIO_TogglePin(Perph* perph, uint16_t pin);

void case_desensitize(char* buff, char pos);
void flush_buffer(char* buff);
char retrieve_function(char* buff);
void execute(char* buff);
void display_help();

void set_leds(char* args, uint16_t state);
void toggle_leds(char* args);
void status_leds(char* args);
void set_blink(char* args);

void set_displayed_value(char* args);
void show_number();
void read_number();

void read_joystick(char* args);

uint32_t GetTick();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char buff[13];
	unsigned char pos = 0;

	uint32_t tickstart;
	bool wait = false;
	char i = 9;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  System_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  Clock_Config();

  /* USER CODE BEGIN SysInit */
  GPIO_Config();
  RCC->CCIPR |= LPUART1_SRC_PCLK;
  LPUART1_CLK_ENABLE();
  LPUART_init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  LPUART_SendString("Simple menu by Arseni Skrabneu\n\r\0");
  LPUART_SendString("Write 'help' to obtain more information\n\r>\0");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(pos == 0)
	  {
		  flush_buffer(buff);

	  }
	  if(!LPUART_ReceiveChar(&buff[pos])) //if a character was successfully received
	  {
		  LPUART_SendChar(buff[pos]);

		  if(buff[pos] == '\r')
		  {
			  LPUART_SendChar('\n');
			  case_desensitize(buff, pos);
			  execute(buff);
			  pos = 0;
			  LPUART_SendChar('>');
		  }
		  else
		  {
			  if(buff[pos] == 127 || buff[pos] == 8) //if backspace was sent
			  {
				  buff[pos] = '\0';
				  if(pos > 0)
				  {
					  buff[pos - 1] = '\0';
				  }
				  (pos > 1)?( pos-- ):( pos = 0 );
			  }
			  else
			  {
				  pos = (pos + 1)%13;
			  }
		  }
	  }

	  if(blink)
	  {
		  if(!wait)
		  {
			  if(i > 0)
			  {
				  wait = true;
				  tickstart = GetTick();
				  GPIO_TogglePin(perph, pin);
				  i--;
			  }
			  else
			  {
				  blink = false;
				  i = 9;
				  GPIO_TogglePin(perph, pin);
			  }
		  }
		  else
		  {
				if((GetTick() - tickstart) > (1000/speed))
				{
					wait = false;
				}
		  }
	  }

	  if(show)
	  {
		  show_number();
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
/* USER CODE BEGIN 4 */
void System_Init()
{
	#if (INSTRUCTION_CACHE_ENABLE == 0)
		FLASH_INSTRUCTION_CACHE_DISABLE();
	#endif

	#if (DATA_CACHE_ENABLE == 0)
		FLASH_DATA_CACHE_DISABLE();
	#endif

	#if (PREFETCH_ENABLE != 0)
		FLASH_PREFETCH_BUFFER_ENABLE();
	#endif

	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    SYSCFG_CLK_ENABLE();
    PWR_CLK_ENABLE();
}

void Osc_Init(Osc_InitStruct* initstruct)
{
	if(initstruct->OscType == OSCILLATORTYPE_MSI)
	{
		RCC->CR |= initstruct->Range & 0x1UL;
		RCC->CR |= (initstruct->Range & 0xFFFEUL) >> 1;
		RCC->CR |= initstruct->State;
	}
}

void Clk_Init(Clk_InitStruct* initstruct)
{
	RCC->CFGR |= initstruct->SYSCLKSource;
	RCC->CFGR |= (initstruct->HCLK_Div | initstruct->PCLK1_Div);
}

void Tim_Init(Tim_InitStruct* initstruct)
{
	TIM2_CLK_ENABLE();
	TIM2->CR1 |= (initstruct->Dir | initstruct->Mode | (uint32_t) (1 << 7));
	TIM2->PSC |= initstruct->Prescaler;
	TIM2->ARR &= 0x0UL;
	TIM2->ARR = initstruct->Reload;
	TIM2->DIER |= TIM2_DIER_UIE;

	NVIC_SetPriority(TIM2_IRQn, 0x0); // Enable the NVIC interrupt for TIM2.
	NVIC_EnableIRQ(TIM2_IRQn);

	TIM2->EGR |= TIM_EGR_UG;
	TIM2->CR1 |= initstruct->State;
}

void Clock_Config(void)
{
	Osc_InitStruct Init_Struct1 = {0};
	Clk_InitStruct Init_Struct2 = {0};
	Tim_InitStruct Init_Struct3 = {0};

	Init_Struct1.OscType = OSCILLATORTYPE_MSI;
	Init_Struct1.Range = (MSI_CLK_RANGE6 << 1)|MSI_CLK_RANGE_SEL_CR;
	Init_Struct1.State = MSI_ON;

	Osc_Init(&Init_Struct1);

	Init_Struct2.SYSCLKSource = SYSCLK_SOURCE_MSI;
	Init_Struct2.HCLK_Div = AHB_CLK_PRESCALER1;
	Init_Struct2.PCLK1_Div = APB1_CLK_PRESCALER1;

	Clk_Init(&Init_Struct2);

	Init_Struct3.Dir = TIM2_CNT_UP;
	Init_Struct3.Mode = TIM2_CNT_EAM;
	Init_Struct3.Prescaler = TIM2_CNT_PSC;
	Init_Struct3.Reload = TIM2_CNT_RELOAD;
	Init_Struct3.State = TIM2_CNT_ON;

	Tim_Init(&Init_Struct3);
}

void GPIO_Init(Perph* perph, Pin_InitStruct* initstruct)
{
	uint32_t pos = 0;
	uint32_t cur = 0;
	while( (initstruct->Pin >> pos) > 0 )
	{
		cur = initstruct->Pin & (1 << pos);
		if(cur != 0)
		{
			perph->MODER = (perph->MODER & ~(0x3UL << (2 * pos))) | (((initstruct->Mode & 0xFFFFFFFEUL) >> 1) << (2 * pos));
			perph->OTYPER = (perph->OTYPER & ~(0x1UL << pos)) | ((initstruct->Mode & 0x01UL) << pos);
			perph->OSPEEDR = (perph->OSPEEDR & ~(0x3UL << (2 * pos))) | (initstruct->Speed << (2 * pos));
			perph->PUPDR = (perph->PUPDR & ~(0x3UL << (2 * pos))) | (initstruct->Pull << (2 * pos));
			if(pos < 8)
			{
				perph->AFR[0] |= (initstruct->Alternate << 4 * pos);
			}
			else
			{
				perph->AFR[1] |= (initstruct->Alternate << 4 * (pos - 8));
			}
		}
		pos++;
	}
}

void GPIO_Config()
{
	PWR_VDDIO2_ENABLE();
	Pin_InitStruct Init_Struct = {0};

	GPIOB_CLK_ENABLE();
	GPIOC_CLK_ENABLE();
	GPIOD_CLK_ENABLE();
	GPIOE_CLK_ENABLE();
	GPIOG_CLK_ENABLE();

	//GPIOB peripheral init start
	Init_Struct.Pin = (1 << 12)|(1 << 13);
	Init_Struct.Mode = GPIO_MODE_ALTER_PP;
	Init_Struct.Alternate = 0x8UL;

	GPIO_Init(GPIOB, &Init_Struct);

	Init_Struct.Pin = (1 << 2)|(1 << 3)|(1 << 4)|(1 << 5)|(1 << 8);
	Init_Struct.Mode = GPIO_MODE_OUTPUT_PP;

	GPIO_Init(GPIOB, &Init_Struct);
	//GPIOB peripheral init end

	//GPIOC peripheral init start
	Init_Struct.Pin = (1 << 0)|(1 << 1);
	Init_Struct.Mode = GPIO_MODE_ALTER_PP;
	Init_Struct.Alternate = 0x8UL;

	GPIO_Init(GPIOC, &Init_Struct);

	Init_Struct.Pin = (1 << 6)|(1 << 7)|(1 << 8)|(1 << 9);
	Init_Struct.Mode = GPIO_MODE_OUTPUT_PP;

	GPIO_Init(GPIOC, &Init_Struct);
	//GPIOC peripheral init end

	//GPIOD peripheral init start
	Init_Struct.Pin = (1 << 3)|(1 << 12)|(1 << 13);
	Init_Struct.Mode = GPIO_MODE_OUTPUT_PP;

	GPIO_Init(GPIOD, &Init_Struct);
	//GPIOD peripheral init end

	//GPIOE peripheral init start
	Init_Struct.Pin = (1 << 0)|(1 << 1)|(1 << 2)|(1 << 15);
	Init_Struct.Mode = GPIO_MODE_INPUT;

	GPIO_Init(GPIOE, &Init_Struct);

	Init_Struct.Pin = (1 << 4)|(1 << 5)|(1 << 6);
	Init_Struct.Mode = GPIO_MODE_OUTPUT_PP;

	GPIO_Init(GPIOE, &Init_Struct);
	//GPIOE peripheral init end

	//GPIOG peripheral init start
	Init_Struct.Pin = (1 << 0)|(1 << 1)|(1 << 2)|(1 << 3)|(1 << 4)|(1 << 5)|(1 << 6)|(1 << 9);
	Init_Struct.Mode = GPIO_MODE_OUTPUT_PP;

	GPIO_Init(GPIOG, &Init_Struct);
	//GPIOG peripheral init end
}

uint32_t GPIO_ReadPin(Perph* perph, uint16_t pin)
{
	int pos = 0;
	while(pin != 0x1U)
	{
		pin = pin >> 1;
		pos++;
	}
	if(((perph->MODER & (0x3UL << 2 * pos)) >> (2 * pos)) == 0x01UL)
	{
		return ((perph->ODR & (0x1UL << pos)) >> pos);
	}
	return ((perph->IDR & (0x1UL << pos)) >> pos);
}

void GPIO_WritePin(Perph* perph, uint16_t pin, uint16_t pinstate)
{
	uint16_t temp = pin;
	uint16_t mask = 0x0U;
	int pos = 0;
	while(temp > 0)
	{
		while((temp & 0x1U) != 0x1U)
		{
			temp = temp >> 1;
			pos++;
		}
		if(((perph->MODER & (0x3UL << 2 * pos)) >> (2 * pos)) == 0x01UL)
		{
			mask |= (1 << pos);
		}
		temp &= 0xFFFEU;
	}
	pin &= mask;
	perph->ODR &= ~pin;
	if(pinstate == 0x1U)
	{
		perph->ODR |= pin;
	}
}

void GPIO_TogglePin(Perph* perph, uint16_t pin)
{
	uint16_t temp = pin;
	uint16_t pinstate = 0x0U;
	int pos = 0;
	while(temp > 0)
	{
		while((temp & 0x1U) != 0x1U)
		{
			temp = temp >> 1;
			pos++;
		}
		pinstate = GPIO_ReadPin(perph, 1 << pos)^1;
		temp &= 0xFFFEU;
	}
	GPIO_WritePin(perph, pin, pinstate);
}

void case_desensitize(char* buff, char pos)
{
	for(char i = 0; i < pos; i++)
	{
		if(*(buff + i) >= 65 && *(buff + i) <= 90)
		{
			*(buff + i) += 32;
		}
	}
}

void flush_buffer(char* buff)
{
	for(char i = 0; i < 13; i++)
	{
		*(buff + i) = '\0';
	}
}

char retrieve_function(char* buff)
{
	unsigned char i, end1, end2;
	unsigned char command;
	char *commands[13] = {
			"help\0",
			"led\0",
			"7led\0",
			"joy\0",
			"lpuart\0",
			"up\0",
			"set\0",
			"clear\0",
			"blink\0",
			"toggle\0",
			"display\0",
			"read\0",
			"status\0"
	};
	for(unsigned char k = 0; k < 13; k++)
	{
		command = k;
		i = 0;
		end1 = 0;
		end2 = 0;
		while(1)
		{
			if(*(commands[k] + i) == '\0')
			{
				end1 = 1;
			}
			if(*(buff + i) == ' ' || *(buff + i) == '\r')
			{
				end2 = 1;
			}

			if(end1^end2)
			{
				command = Invalid;
				break;
			}
			else if(end1&end2)
			{
				break;
			}
			else if(*(commands[k] + i) != *(buff + i))
			{
				command = Invalid;
				break;
			}
			i++;
		}
		if(command != Invalid)
		{
			return command;
		}
	}
	return Invalid;
}

void execute(char* buff)
{
	unsigned char command;
	if((command = retrieve_function(buff)) != Invalid)
	{
		unsigned char i = 0;
		char args[4] = {'\0','\0','\0','\0'};
		while(*(buff + i) != ' '){ i++; }
		i++;
		for(unsigned char k = i; k < 13; k++)
		{
			if(*(buff + k) == '\r')
			{
				break;
			}
			args[k - i] = *(buff + k);
		}
		switch(level)
		{
			case __Menu:
				switch(command)
				{
					case HELP:
						display_help();
						break;
					case LED:
						level = __LED;
						break;
					case LED7:
						level = __LED7;
						break;
					case Joy:
						level = __Joy;
						break;
					case LPUART:
						level = __LPUART;
						break;
					default:
						LPUART_SendString("You cannot use this command here! Type \"HELP\" for more information.\n\r\0");
						break;
				}
				break;
			case __LED:
				switch(command)
				{
					case HELP:
						display_help();
						break;
					case UP:
						level = __Menu;
						break;
					case Set:
						set_leds(args, 0x1U);
						break;
					case Clear:
						set_leds(args, 0x0U);
						break;
					case Blink:
						set_blink(args);
						break;
					case Status:
						status_leds(args);
						break;
					case Toggle:
						toggle_leds(args);
						break;
					default:
						LPUART_SendString("You cannot use this command here! Type \"HELP\" for more information.\n\r\0");
						break;
				}
				break;
			case __LED7:
				switch(command)
				{
					case HELP:
						display_help();
						break;
					case UP:
						level = __Menu;
						break;
					case Display:
						set_displayed_value(args);
						break;
					case Read:
						read_number();
						break;
					default:
						LPUART_SendString("You cannot use this command here! Type \"HELP\" for more information.\n\r\0");
						break;
				}
				break;
			case __Joy:
				switch(command)
				{
					case HELP:
						display_help();
						break;
					case UP:
						level = __Menu;
						break;
					case Read:
						read_joystick(args);
						break;
					default:
						LPUART_SendString("You cannot use this command here! Type \"HELP\" for more information.\n\r\0");
						break;
				}
				break;
			case __LPUART:
				switch(command)
				{
					case HELP:
						display_help();
						break;
					case UP:
						level = __Menu;
						break;
					case Status:
						lpuart_status();
						break;
					default:
						LPUART_SendString("You cannot use this command here! Type \"HELP\" for more information.\n\r\0");
						break;
				}
				break;
		}
	}
	else
	{
		LPUART_SendString("Is not a valid command! Type \"HELP\" for more information.\n\r\0");
	}
}

void display_help()
{
	switch(level)
	{
		case __Menu:
			LPUART_SendString("Top level\n\r\0");
			LPUART_SendString("(LED|7LED|Joy|LPUART)\tGo to the respective level\n\r\0");
			break;
		case __LED:
			LPUART_SendString("UP\tGo up one level\n\r\0");
			LPUART_SendString("Set <id>\tTurn on LED (id: 0-7 or R,G,B)\n\r\0");
			LPUART_SendString("Clear <id>\tTurn off LED (id: 0-7 or R,G,B)\n\r\0");
			LPUART_SendString("Blink <id>\tBlink LED five times (id: 0-7 or R,G,B)\n\r\0");
			LPUART_SendString("Status <id>\tDisplay on/off status of LED (id: 0-7 or R,G,B)\n\r\0");
			LPUART_SendString("Toggle <id>\tInvert LED state (id: 0-7 or R,G,B)\n\r\0");
			break;
		case __LED7:
			LPUART_SendString("UP\tGo up one level\n\r\0");
			LPUART_SendString("Display <val>\tDisplay number from 0 to 9999 on the 7-LED display\n\r\0");
			LPUART_SendString("Read\tDisplay on the terminal the number from 7-LED display\n\r\0");
			break;
		case __Joy:
			LPUART_SendString("UP\tGo up one level\n\r\0");
			LPUART_SendString("Read <id>\tDisplay on the terminal the current state of joystick buttons\n\r\0");
			LPUART_SendString("\t\t(id: L – left, R – right, U – up, D – down, C - center)\n\r\0");
			break;
		case __LPUART:
			LPUART_SendString("UP\tGo up one level\n\r\0");
			LPUART_SendString("Status\tDisplay baudrate, number of databits and parity bits,\n\r\0");
			LPUART_SendString("\t\tinformations read from registers and then calculated baudrate\n\r\0");
			break;
	}
}

void set_leds(char* args, uint16_t state)
{
	if(*(args + 0) >= 48 && *(args + 0) <= 55)
	{
		GPIO_WritePin(leds[*(args + 0) - 48].perph, leds[*(args + 0) - 48].number, state);
	}
	else if(*(args + 0) == 'r' || *(args + 0) == 'g' || *(args + 0) == 'b')
	{
		switch(*(args + 0))
		{
			case 'r':
				GPIO_WritePin(leds[8].perph, leds[8].number, state);
				break;
			case 'g':
				GPIO_WritePin(leds[9].perph, leds[9].number, state);
				break;
			case 'b':
				GPIO_WritePin(leds[10].perph, leds[10].number, state);
				break;
		}
	}
	else
	{
		LPUART_SendString("Not a valid argument! Type \"HELP\" for more information.\n\r\0");
	}
}

void toggle_leds(char* args)
{
	if(*(args + 0) >= 48 && *(args + 0) <= 55)
	{
		GPIO_TogglePin(leds[*(args + 0) - 48].perph, leds[*(args + 0) - 48].number);
	}
	else if(*(args + 0) == 'r' || *(args + 0) == 'g' || *(args + 0) == 'b')
	{
		switch(*(args + 0))
		{
			case 'r':
				GPIO_TogglePin(leds[8].perph, leds[8].number);
				break;
			case 'g':
				GPIO_TogglePin(leds[9].perph, leds[9].number);
				break;
			case 'b':
				GPIO_TogglePin(leds[10].perph, leds[10].number);
				break;
		}
	}
	else
	{
		LPUART_SendString("Not a valid argument! Type \"HELP\" for more information.\n\r\0");
	}
}

void status_leds(char* args)
{
	bool state;
	if(*(args + 0) >= 48 && *(args + 0) <= 55)
	{
		LPUART_SendString("Status of LED \0");
		LPUART_SendChar(*(args + 0));
		LPUART_SendString(" : \0");
		state = GPIO_ReadPin(leds[*(args + 0) - 48].perph, leds[*(args + 0) - 48].number);
		if(state)
		{
			LPUART_SendString("On\n\r\0");
		}
		else
		{
			LPUART_SendString("Off\n\r\0");
		}
	}
	else if(*(args + 0) == 'r' || *(args + 0) == 'g' || *(args + 0) == 'b')
	{
		LPUART_SendString("Status of LED \0");
		LPUART_SendChar(*(args + 0) - 32);
		LPUART_SendString(" : \0");
		switch(*(args + 0))
		{
			case 'r':
				state = GPIO_ReadPin(leds[8].perph, leds[8].number);
				break;
			case 'g':
				state = GPIO_ReadPin(leds[9].perph, leds[9].number);
				break;
			case 'b':
				state = GPIO_ReadPin(leds[10].perph, leds[10].number);
				break;
		}
		if(state)
		{
			LPUART_SendString("On\n\r\0");
		}
		else
		{
			LPUART_SendString("Off\n\r\0");
		}
	}
	else
	{
		LPUART_SendString("Not a valid argument! Type \"HELP\" for more information.\n\r\0");
	}
}

void set_blink(char* args)
{
	blink = true;
	if(*(args + 0) >= 48 && *(args + 0) <= 55)
	{
		perph = leds[*(args + 0) - 48].perph;
		pin = leds[*(args + 0) - 48].number;
	}
	else if(*(args + 0) == 'r' || *(args + 0) == 'g' || *(args + 0) == 'b')
	{
		switch(*(args + 0))
		{
			case 'r':
				perph = leds[8].perph;
				pin = leds[8].number;
				break;
			case 'g':
				perph = leds[9].perph;
				pin = leds[9].number;
				break;
			case 'b':
				perph = leds[10].perph;
				pin = leds[10].number;
				break;
		}
	}
	else
	{
		LPUART_SendString("Not a valid argument! Type \"HELP\" for more information.\n\r\0");
		return;
	}
}

void set_displayed_value(char* args)
{
	if(!show)
	{
		show = true;
	}
	for(unsigned char i = 0; i < 4; i++)
	{
		count[i] = '\0';
	}

	for(unsigned char i = 0; i < 4; i++)
	{
		if(*(args + i) == '\0')
		{
			break;
		}
		if(*(args + i) < 48 || *(args + i) > 57)
		{
			LPUART_SendString("Not a valid argument! Type \"HELP\" for more information.\n\r\0");
			return;
		}
	}
	for(unsigned char i = 0; i < 4; i++)
	{
		if(*(args + i) == '\0')
		{
			break;
		}
		count[i] = *(args + i);
	}
}

void show_number()
{
	bool wait = false;
	int i = 4;
	while(count[i - 1] == '\0') {i--;}
	for(int digit = i; digit > 0; digit--)
	{
		wait = !wait;
		uint32_t tickstart = GetTick();
		GPIO_WritePin(GPIOG, *(digits + (count[digit - 1] - 48)*sizeof(uint16_t)/2), 0x1U);
		GPIO_WritePin(GPIOB, ((uint16_t) 1 << (5 - i + digit)), 0x1U);
		while(wait)
		{
			if(GetTick()-tickstart >= 5)
			{
				wait = !wait;
				GPIO_WritePin(GPIOG, ((uint16_t) 1 << 16) - 1, 0x0U);
				GPIO_WritePin(GPIOB, ((uint16_t) 1 << (5 - i + digit)), 0x0U);
			}
		}
	}
}

void read_number()
{
	LPUART_SendString("Displayed number: \0");
	if(show)
	{
		for(unsigned char i = 0; i < 4; i++)
		{
			if(count[i] == '\0')
			{
				break;
			}
			LPUART_SendChar(count[i]);
		}
	}
	else
	{
		LPUART_SendString("None\0");
	}
	LPUART_SendString("\n\r\0");
}

void read_joystick(char* args)
{
	uint32_t state;
	if(*(args + 0) == 'l' || *(args + 0) == 'r' || *(args + 0) == 'u' || *(args + 0) == 'd' || *(args + 0) == 'c')
	{
		switch(*(args + 0))
		{
			case 'r':
				state = GPIO_ReadPin(GPIOE, (uint16_t) (1 << 0));
				break;
			case 'l':
				state = GPIO_ReadPin(GPIOE, (uint16_t) (1 << 1));
				break;
			case 'u':
				state = GPIO_ReadPin(GPIOE, (uint16_t) (1 << 3));
				break;
			case 'd':
				state = GPIO_ReadPin(GPIOE, (uint16_t) (1 << 2));
				break;
			case 'c':
				state = GPIO_ReadPin(GPIOE, (uint16_t) (1 << 15));
				break;
		}
		LPUART_SendString("State: \0");
		LPUART_SendString((state)?("Off"):("On")); //by default the pin is connected to Vcc
		LPUART_SendString("\n\r\0");
	}
	else
	{
		LPUART_SendString("Not a valid argument! Type \"HELP\" for more information.\n\r\0");
	}
}

uint32_t GetTick()
{
	return glTick;
}

void TIM2_IRQHandler()
{
	glTick++;
	TIM2->SR &= ~(0x1UL << 0);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

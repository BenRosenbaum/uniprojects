/*
 * File:   Major_Project_Test.c
 * Author: benrosenbaum
 *
 * Created on May 6, 2021, 4:07 PM
 */


#include <avr/io.h>
#include <xc.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <string.h>



#define I2C_READ    1
#define I2C_WRITE   0

#define Green  0
#define Yellow 1
#define Red    2
#define White  0
#define Blue   1
#define PRed   1
#define TRed   1

volatile int count=0;
volatile int counter=0;
volatile int current_state=0;
volatile int count_timer2=0;
volatile int millis_timer2=0;
volatile int period=0;
volatile int HSperiod=0;


volatile unsigned char flag=0;
volatile unsigned long millis=0;
volatile unsigned char BSMflag=0;
volatile unsigned char BTflag=0;
volatile unsigned char BNTflag=0;
volatile unsigned char BSTflag=0;
volatile unsigned char BBflag=0;
volatile unsigned char BNMflag=0;
volatile unsigned char LSflag=0;
volatile unsigned char PLSflag=0;
volatile unsigned char PBSflag=0;
volatile unsigned char PBNflag=0;
volatile unsigned char Pflag=0;
volatile unsigned char Tflag=0;


volatile uint8_t  AtoD=0;
volatile uint8_t  addr=0x27;

//NEED TO ALTER BELOW STATE to ensure enough states in the system
//POTENTIALLY ADD (BB BLUE AND BB OFF) and also (PED GREEN  & PED FLASH & PED OFF) AND (TRAM WHITE & TRAM RED)

enum STATE {HS, BB, BCred, BCyellow, BCgreen, BTred, BTyellow, BTgreen, LSred,LSyellow, LSgreen, P, Pflash,Pdont, T, L};
enum STATE state;



/*void timer_2setup(){
    //timer2 for hazard state - fixed time period - 1ms
    cli(); //clear global interrupt flag
    
    OCR2A = 249; //interrupt occurs 1 second second
    TCCR2A|= (1<<WGM00)|(1<<WGM01); //timer control register A
    TCCR2B|= (1<CS00)|(1<CS01)|(1<<CS02); //prescalar set to 256
    
    TCNT2 = 0; //timer counter
    
    //want interrupt
    TIMSK2 |= (1<<OCIE2A); //interrupt mask register - compare A match interrupt enable
    
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    
    sei(); //enable global interrupt
   
}*/

void timer_0setup(){
    //timer0 - 1ms timer
    cli(); //clear global interrupt flag
    
    OCR0A = 249; //interrupt occurs 1ms
    TCCR0A|= (1<<WGM01) | (1<<WGM00); //timer control register A
    TCCR0B|= (1<<CS00) |(1<<CS01)|(1<<CS02); //prescalar set to 256
    
    TCNT0 = 0; //timer counter
    TIFR0 = 0xff; //clear all flags
    //want interrupt
    TIMSK0 |= (1<<OCIE0A); //interrupt mask register - compare A match interrupt enable
    
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    
    sei(); //enable global interrupt
   
}

void timer_1setup(){ 
    //CTC mode
    TCCR1A = 0;
    TCCR1B |= (1<<CS10) | (1<<WGM12);  //CTC mode and prescalar 1
    TCCR1C = 0; 
    TIMSK1 = 0; //interrupt mask register
    OCR1A = 4000; //frequency =2000Hz | need to increment OCR1A to result in descending frequency
    
}



int8_t SPI_transfer(int8_t data) {
  
    SPDR = data;
    while ((SPSR & _BV(SPIF)) == 0) {
        ;   // wait until transfer completed
    }
    return SPDR;
}

void SPI_Send_Command(uint8_t chip_id, uint8_t reg, uint8_t data) {
    // Send a command + byte to SPI interface
    if (chip_id==2){
        PORTB&= ~_BV(0);
    } else {
    PORTB &= ~_BV(2);    // SS enabled (low))
}
    SPI_transfer(0x40);
    SPI_transfer(reg);
    SPI_transfer(data);
    PORTB |= _BV(2);    // SS disabled (high)
}

uint8_t SPI_Read_Command(int chip_id, uint8_t reg) {
    uint8_t data;
    
    // Read a command output from SPI interface
    // SS enabled (low))
    if (chip_id == 1){
        PORTB &= ~_BV(2);    
    }else{
        PORTB &= ~_BV(0);
    }
    SPI_transfer(0x41);
    SPI_transfer(reg);
    data = SPI_transfer(0);
    
    // SS disabled (high)
    if (chip_id == 1){
        PORTB |= _BV(2);    
    }else{
        PORTB |= _BV(0);
    }
    return data;
}

void setup_SPI() {
    DDRB  = 0b00101111;
	PORTB = 0b00000101;		// set SS signal high
    DDRD  = 0b00000010; // allow UART to still be default
    PORTD = 0b11111101; // turn on pullups for all inputs
    DDRC  = 0b00001110; // setup LEDs as outputs and rest as inputs
    PORTC = 0b00110000; // do NOT turn on pullup for analog input

	SPCR = _BV(SPE)|_BV(MSTR);   // set master SPI, SPI mode 0 operation
	SPSR = 0;                    // clear interrupt flags and oscillator mode.


    // Now that the SPI interface is configured we need to send SPI commands to configure the MCP23S17
    // port expander IC
    // Configure Expander A
    SPI_Send_Command(1, 0x00, 0x00);   // register IODIRA (port A data direction)
    SPI_Send_Command(1, 0x01, 0xff);   // register IODIRB (port B data direction)
    SPI_Send_Command(1, 0x0d, 0xff);   // register GPPUB (port B GPIO Pullups)
    SPI_Send_Command(1, 0x05, 0xff);   // register GPINTENB (enable port B interrupts)
    SPI_Send_Command(1, 0x09, 0x00);   // register INTCONB (port B interrupt on change)

    // Configure Expander B
    SPI_Send_Command(2, 0x00, 0x00);   // register IODIRA (port A data direction)
    SPI_Send_Command(2, 0x01, 0x0f);   // register IODIRB (port B data direction)
    SPI_Send_Command(2, 0x0d, 0x0f);   // register GPPUB (port B GPIO Pullups)
    SPI_Send_Command(2, 0x05, 0x0f);   // register GPINTENB (enable port B interrupts)
    SPI_Send_Command(2, 0x09, 0x00);   // register INTCONB (port B interrupt on change)    

    
}

void setup_AtoD() {
    ADMUX = 0b01100000;     // Avcc, ADLAR = 1, channel ADC0
            // enable, auto retrigger, interrupt scale = 64
    ADCSRA = _BV(ADEN) | _BV(ADATE) | _BV(ADIE) |  0b110;
    ADCSRB = 0b00000000;
    ADCSRA |= _BV(ADSC);   // start the conversions. Will auto restart
}

ISR(ADC_vect) {
    AtoD = ADCH;
    //value = readAtoD();
}

uint8_t read_AtoD() {
    //analog to digital conversion
    if ((ADCSRA & _BV(ADSC)) == 0) {
        ADCSRA |= _BV(ADSC);    // start a conversion
    }
    while ((ADCSRA & _BV(ADSC)) != 0) {
        ;   // wait for the conversion to finish
    }
    return ADCH;
}

void setup_I2C() {
    DDRC &= 0x0f;
    PORTC|= 0x30;  // Port C pins 4 and 5 set to input with pullups
    TWBR = 193;
    TWSR = 0;
}

/**
 * Wait for the current I2C operation to finish.
 * This possible allows the processor to wait forever, but if the I2C bus
 * is enabled it will eventually finish.
 */
void I2C_wait() {
    while ((TWCR & _BV(TWINT)) == 0) {
        ;
    }
}

/**
 * Send an I2C start bit.
 * 
 * @return true if the start bit was successfully transmitted
 */
int I2C_Start() {
    // Send I2C Start flag
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
    I2C_wait();
    return ((TWSR & 0xf8) == 0x08);
}

/**
 * Send an I2C address byte and R/W flag
 * 
 * @param addr I2C address of the slave
 * @param rw whether to read or write: 0 to write, 1 to read
 * @return true if the address byte was successfully transmitted
 */
int I2C_SLA(uint8_t addr, uint8_t rw) {
    // Send I2C slave address
    TWDR = (addr << 1) | (rw & 1);
    TWCR = _BV(TWINT) | _BV(TWEN);
    I2C_wait();
    return ((TWSR & 0xf8) == 0x18);
}

/**
 * Send a byte of data through the I2C bus
 * 
 * @param data data to transmit
 * @return true if the data was successfully transmitted
 */
int I2C_Send(uint8_t data) {
    // Send I2C data byte
    TWDR = data;
    TWCR = _BV(TWINT) | _BV(TWEN);
    I2C_wait();
    return ((TWSR & 0xf8) == 0x28);
}

/**
 * Send the stop flag on the I2C bus
 */
void I2C_Stop() {
    // Send I2C Stop flag
    TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
    for (volatile long x = 0; x < 100; x++) {
        ;
    }
}



/**
 * Send four bits of data to a PCF8574 controlled HD44780 LCD display
 * We need to toggle the E bit (bit 2) from high to low to transmit the data
 * 
 * The 8 bits transmitted are:
 * bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0
 * DB7  DB6  DB5  DB4  BL   E    R/W  RS
 * BL is the back light (1 = on, 0 = off)
 * E is the enable bit (high to low transition latches the data
 * R/W is the read/write line (1 = read, 0 = write)
 * RS is Register Select (0 = control, 1 = data)
 * 
 * @param data the data to transmit
 * @return true if the data was transmitted
 */
int I2C_PCF8574_LCD_Nibble(uint8_t data) {
    TWDR = data | 0x04;
    TWCR = _BV(TWINT) | _BV(TWEN);
    I2C_wait();
    if ((TWSR & 0xf8) == 0x28) {
        TWDR = data & (~0x04);
        TWCR = _BV(TWINT) | _BV(TWEN);
        I2C_wait();
    }
    return ((TWSR & 0xf8) == 0x28);
}

/**
 * Transmit the 8 bits of data as two four bit nibbles to a HD44780 LCD
 * controller in 4 bit mode attached through a PCF8574 port expander.
 * 
 * The byte is transmitted as the top nibble then the bottom nibble with
 * the bottom four bits being the control flags.
 * 
 * @param data 8 bits of data to transmit
 * @param flags 4 bits if flags
 * @return true if the data was transmitted
 */
int I2C_PCF8574_LCD_Byte(uint8_t data, uint8_t flags) {
    return I2C_PCF8574_LCD_Nibble ((data & 0xf0) | (flags & 0x0f)) && 
    I2C_PCF8574_LCD_Nibble (((data << 4) & 0xf0) | (flags & 0x0f));
}

/**
 * Send multiple bytes of data to the LCD display
 * 
 * @param addr address of the display
 * @param array pointer to a char array of data to transmit
 * @param len number of bytes in the array
 * @param flags the flags to transmit as the lower 4 bits
 */
void I2C_SendData(uint8_t addr, uint8_t *array, uint8_t len, uint8_t flags) {
    if (I2C_Start() & I2C_SLA(addr, I2C_WRITE)) {
        while (len > 0) {
            len--;
            if (I2C_PCF8574_LCD_Byte(*array++, flags) == 0) {
                break;  // bad send
            }
        }
    }
    I2C_Stop(); 
}

/**
 * Send the initialisation string for a HD44780 LCD controller connected in
 * 4 bit mode. Taken from the data sheet. Transmit 0x30 three times to ensure
 * it is in 8 bit mode, then 0x20 to switch to 4 bit mode.
 * We then turn on the blinking cursor, backlight and clear the display.
 * 
 * @param addr address of the LCD display
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_PCF8574_Setup(uint8_t addr) {
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_Send(0);    // ensure the PCF8574 enable line is low
    I2C_Stop();
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Nibble(0x30);
    I2C_Stop();
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Nibble(0x30);
    I2C_Stop();
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Nibble(0x30);
    I2C_Stop();
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Nibble(0x20);
    I2C_Stop();
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Byte(0x0f, 0x00);   // display on, cursor on and blinking
    I2C_Stop();
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Byte(0x01, 0x00);   // clear and move home
    I2C_Stop();
    return 0;
}

/**
 * Clear the LCD display (and return the cursor to the home position
 * 
 * @param addr address of the LCD display
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_clear(uint8_t addr) {
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Byte(0x01, 0x08);   // clear screen command
    I2C_Stop();
    return 0;
}

/**
 * Set the cursor position on the LCD display
 * See the data sheet for mappings of position values to screen locations.
 * (0x00 top left, 0x40 second row left)
 * 
 * @param addr address of the LCD display
 * @param posn Location to where the cursor should be positioned
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_Position(uint8_t addr, uint8_t posn) {
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    I2C_PCF8574_LCD_Byte(0x80 | posn, 0x08);   // set DRAM address
    I2C_Stop();
    return 0;
}

/**
 * Write a string to the LCD display
 * 
 * @param addr address of the LCD display
 * @param str pointer to a character string to display
 * @param len length of the string to display
 * @return -1 if the display doesn't respond to a selection
 */
int8_t LCD_Write(uint8_t addr, char *str, uint8_t len) {
    if (!(I2C_Start() && I2C_SLA(addr, I2C_WRITE))) {
        return -1;
    }
    while (len--) {
        I2C_PCF8574_LCD_Byte(*str++, 0x09);
    }
    I2C_Stop();
    return 0;
}

char hex(int value) {
    return "0123456789ABCDEF"[value & 0x0f];
}

void LCD_hex(uint8_t addr, uint8_t posn, uint8_t num) {
    char hstr[] = "00";
    
    LCD_Position(addr, posn);
    hstr[0] = hex((num >> 4) & 0x0f);
    hstr[1] = hex((num >> 0) & 0x0f);
    LCD_Write(addr, hstr, 2);
}

void LCD_quad(uint8_t addr, uint8_t posn, uint32_t num) {
    char qstr[] = "00000000";
    
    LCD_Position(addr, posn);
    qstr[0] = hex((num >> 28) & 0x0f);
    qstr[1] = hex((num >> 24) & 0x0f);
    qstr[2] = hex((num >> 20) & 0x0f);
    qstr[3] = hex((num >> 16) & 0x0f);
    qstr[4] = hex((num >> 12) & 0x0f);
    qstr[5] = hex((num >> 8) & 0x0f);
    qstr[6] = hex((num >> 4) & 0x0f);
    qstr[7] = hex((num >> 0) & 0x0f);
    LCD_Write(addr, qstr, 8);
}

/**
 * Setup the LCD display
 * 
 * @param addr address of the LCD display
 * @return -1 if the display doesn't respond to a selection
 */
int8_t setup_LCD(uint8_t addr) {
    if (LCD_PCF8574_Setup(addr) != 0) {
        return -1;
    }
    LCD_clear(addr);
    return 0;
}


 
/*void HSLights(){
    //Hazard state - hazard state flashes at 1Hz for 10 seconds
    PORTC^=0x01;
    
}

void BBLights(uint8_t BB){
    //Broadway bus State
    SPI_Send_Command(2,0x12,0x04); //Expander B - Pin: GPA4
}


void BCLights(uint8_t BNM, uint8_t BSM){
    //Broadway car State
    SPI_Send_Command(2,0x12,1<<(5+BNM));//BNM - Expander B - Pins: GPA5 - GPA7
    SPI_Send_Command(1,0x12,1<<(5+BSM));//BSM - Expander A - Pins: GPA5 - GPA7
}


void BTLights(uint8_t BT){
    //Broadway turn State
    SPI_Send_Command(1,0x12,1<<(2+BT));//Expander A - Pins: GPA2 - GPA4 
}

void LSLights(uint8_t LS){
    //Little Street State
    SPI_Send_Command(2,0x13,1<<(6+LS)); //Expander 2 - Pins: GPB5 - GPB7
}

void PLights(uint8_t P){
    //Pedestrian State
    SPI_Send_Command(2,0x12,1<<(P)); //Expander B - Pins: GPA0 - GPA1
}

void TLights(uint8_t T){
    //Tram State
    SPI_Send_Command(1,0x12,1<<(T)); //Expander A - Pins: GPA0 - GPA1
}

*/




int sensorBSM(){
    //Broadway South Main
    return ((SPI_Read_Command(1,0x13) & _BV(0)) == 0); 
}

int sensorBT(){
    //Broadway Turn
    return ((SPI_Read_Command(1,0x13) & _BV(1)) == 0); 
}

int sensorBST(){ 
    //Broadway South Tram
    return ((SPI_Read_Command(1,0x13) & _BV(2)) == 0); 
}

int sensorBB(){
    //Broadway Bus
    return ((SPI_Read_Command(1,0x13) & _BV(3)) == 0); 
}

int sensorBNM(){
    //Broadway North Main
    return ((SPI_Read_Command(1,0x13) & _BV(4)) == 0); 
}

int sensorBNT(){
    //Broadway North Tram
    return ((SPI_Read_Command(1,0x13) & _BV(5)) == 0); 
}

int sensorLS(){
    //Little Street
    return ((SPI_Read_Command(1,0x13) & _BV(6)) == 0); 
}

int sensorPBN(){
    //Pedestrian Broadway North
    return ((SPI_Read_Command(1,0x13) & _BV(7)) == 0); 
}

int sensorPBS(){
    //Pedestrian Broadway South
    return ((SPI_Read_Command(2,0x13) & _BV(0)) == 0); 
}

int sensorPLS(){
    //Pedestrian Little Street
    return ((SPI_Read_Command(2,0x13) & _BV(1)) == 0); 
}

// LCD below
//LCD_position(addr, position)
//LCD_Write(addr, str, len)


void LCD_sensorBSM(uint8_t addr){
    //LCD button Broadway South Main
    char open[1]="O";
    char close[1] = "X";
    
    LCD_Position(addr,0x00);
    if(BSMflag==1){
        LCD_Write(addr,close,1);
    } else { 
        LCD_Write(addr,open,1);
 
    }   
}

void LCD_sensorBT(uint8_t addr){
    //LCD button display Broadway Turn
    char open[1]="O";
    char close[1] = "X";
    
    LCD_Position(addr,0x01);
    if(BTflag==1){
        LCD_Write(addr,close,1);
    } else { 
        LCD_Write(addr,open,1);
 
    }   
}

void LCD_sensorBST(uint8_t addr){
    //LCD button display Broadway South Tram
    char open[1]="O";
    char close[1] = "X";
    
    LCD_Position(addr,0x02);
    if(BSTflag==1){
        LCD_Write(addr,close,1);
    } else { 
        LCD_Write(addr,open,1);
 
    }   
}

void LCD_sensorBB(uint8_t addr){
    //LCD button display Broadway Bus
    char open[1]="O";
    char close[1] = "X";
    
    LCD_Position(addr,0x03);
    if(BBflag==1){
        LCD_Write(addr,close,1);
    } else { 
        LCD_Write(addr,open,1);
 
    }   
}

void LCD_sensorBNM(uint8_t addr){
    //LCD button display Broadway North Main
    char open[1]="O";
    char close[1] = "X";
    
    LCD_Position(addr,0x04);
    if(BNMflag==1){
        LCD_Write(addr,close,1);
    } else { 
        LCD_Write(addr,open,1);
 
    }   
}

void LCD_sensorBNT(uint8_t addr){
    //LCD button display Broadway North Tram
    char open[1]="O";
    char close[1] = "X";
    
    LCD_Position(addr,0x05);
    if(BNTflag==1){
        LCD_Write(addr,close,1);
    } else { 
        LCD_Write(addr,open,1);
 
    }   
}

void LCD_sensorLS(uint8_t addr){
    //LCD button display Little Street
    char open[1]="O";
    char close[1] = "X";
    
    LCD_Position(addr,0x06);
    if(LSflag==1){
        LCD_Write(addr,close,1);
    } else { 
        LCD_Write(addr,open,1);
 
    }   
}

void LCD_sensorPBN(uint8_t addr){
    //LCD button display Pedestrian Broadway North
    char open[1]="O";
    char close[1] = "X";
    
    LCD_Position(addr,0x07);
    if(PBNflag==1){
        LCD_Write(addr,close,1);
    } else { 
        LCD_Write(addr,open,1);
 
    }   
}

void LCD_sensorPBS(uint8_t addr){
    //LCD button display Pedestrian Broadway South
    char open[1]="O";
    char close[1] = "X";
    
    LCD_Position(addr,0x08);
    if(PBSflag==1){
        LCD_Write(addr,close,1);
    } else { 
        LCD_Write(addr,open,1);
 
    }   
}

void LCD_sensorPLS(uint8_t addr){
    //LCD button display Pedestrian Little Street
    char open[1]="O";
    char close[1] = "X";
    
    LCD_Position(addr,0x09);
    if(PLSflag==1){
        LCD_Write(addr,close,1);
    } else { 
        LCD_Write(addr,open,1);
 
    }   
}




/*
LCD_SubState(uint8_t addr){
    char HS[]="HS"; //Hazard State
    char BB[]="BB"; //Broadway Bus
    char BC[]="BC"; //Broadway Car
    char BT[]="BT"; //Broadway Turn
    char LS[]="LS"; //Little Street
    char P[] ="P "; //Pedestrian
    char T[] ="T "; //Tram
}*/


void LCD_State(uint8_t addr){//maybe place this in main
    
    char Haz[]="HS"; //Hazard State
    char BBus[]="BB"; //Broadway Bus
    char BC[]="BC"; //Broadway Car
    char BT[]="BT"; //Broadway Turn
    char LS[]="LS"; //Little Street
    char Ped[] ="P "; //Pedestrian
    char Tram[] ="T "; //Tram
    
    LCD_Position(addr, 0x4D); //second last character on the second line
    if(state==HS){
        LCD_Write(addr,Haz,2);
    } if (state==BB){
        LCD_Write(addr,BBus,2);
    } if(state==BCgreen||state==BCyellow||state==BCred){
        LCD_Write(addr,BC,2);
    } if(state==BTgreen||state==BTyellow||state==BTred){
        LCD_Write(addr,BT,2);
    } if(state==LSgreen||state==LSyellow||state==LSred){
        LCD_Write(addr,LS,2);
    } if(state==P||state==Pflash){
        LCD_Write(addr,Ped ,2);
    } if(state==T){
        LCD_Write(addr,Tram ,2);
    }
}
 
    
void LCD_Potentiometer(uint8_t addr){
   // LCD_Position(addr, 0x40); //first character second line
    //determine if LCD write and position required
    LCD_hex(addr, 0x40, AtoD);
    
}

void HazS(){
    //Hazard state - hazard state flashes at 1Hz for 10 seconds
    PORTC^=0x01;
    
} 
    void BBus(){
        //Broadway Bus LED's
  SPI_Send_Command(1, 0x12, 0b10010001); 
  SPI_Send_Command(2, 0x12, 0b10010110); 
  SPI_Send_Command(2, 0x13, 0b10000000); 
}
void BCg(){
    //Broadway Car Green
  SPI_Send_Command(1, 0x12, 0b00110001); 
  SPI_Send_Command(2, 0x12, 0b00100110); 
  SPI_Send_Command(2, 0x13, 0b10000000);  
}

void BCy(){
    //Broadway Car Yellow
   SPI_Send_Command(1, 0x12, 0b00110010); 
   SPI_Send_Command(2, 0x12, 0b01001010); 
   SPI_Send_Command(2, 0x13, 0b10000000);
}

void BCr(){
    //Broadway Car Red
    SPI_Send_Command(1, 0x12, 0b00101010); 
    SPI_Send_Command(2, 0x12, 0b10010010); 
    SPI_Send_Command(2, 0x13, 0b10000000); 
}

void BTg(){
    //Broadway Turn Green
    SPI_Send_Command(1, 0x12, 0b00100110); 
    SPI_Send_Command(2, 0x12, 0b10001010); 
    SPI_Send_Command(2, 0x13, 0b10000000); 
}

void BTy(){
    //Broadway Turn Yellow
    SPI_Send_Command(1, 0x12, 0b01001010); 
    SPI_Send_Command(2, 0x12, 0b10001010); 
    SPI_Send_Command(2, 0x13, 0b10000000); 
}

void BTr(){
    //Broadway Turn Red
    SPI_Send_Command(1, 0x12, 0b10010010); 
    SPI_Send_Command(2, 0x12, 0b10001010); 
    SPI_Send_Command(2, 0x13, 0b10000000);
}

void LSg(){
    //Little Street Green
  SPI_Send_Command(1, 0x12, 0b10010010);
  SPI_Send_Command(2, 0x12, 0b10001010);
  SPI_Send_Command(2, 0x13, 0b00100000); 
}

void LSy(){
    //Little Street Yellow
  SPI_Send_Command(1, 0x12, 0b10010010);
  SPI_Send_Command(2, 0x12, 0b10001010);
  SPI_Send_Command(2, 0x13, 0b01000000); 
}

void LSr(){
    //Little Street Red
  SPI_Send_Command(1, 0x12, 0b10010010);
  SPI_Send_Command(2, 0x12, 0b10001010);
  SPI_Send_Command(2, 0x13, 0b10000000); 
}

void Ped(){
    //Pedestrian
  SPI_Send_Command(1, 0x12, 0b10010010);
  SPI_Send_Command(2, 0x12, 0b10001001);
  SPI_Send_Command(2, 0x13, 0b10000000); 
}

void PF(){
    //Pedestrian Flash
  SPI_Send_Command(1, 0x12, 0b10010010);
  SPI_Send_Command(2, 0x12, 0b10001001);
  SPI_Send_Command(2, 0x13, 0b10000000); 
}
void PD(){
    //Dont walk
  SPI_Send_Command(1, 0x12, 0b10010010);
  SPI_Send_Command(2, 0x12, 0b10001010);
  SPI_Send_Command(2, 0x13, 0b10000000); 
}
void Tram(){
    //Tram State
  SPI_Send_Command(1, 0x12, 0b10010001);
  SPI_Send_Command(2, 0x12, 0b10000110);
  SPI_Send_Command(2, 0x13, 0b10000000); 
}

int HazButton(){
    return((PIND & _BV(7))==0);
}
                  
ISR(INT0_vect){
     if(sensorBSM()){
        BSMflag=1;
    }
    if(sensorBT()){
        BTflag=1;
    }
  
    if(sensorBB()){
        BBflag=1;
    }
    if(sensorBNM()){
        BNMflag=1;
    }
     
     if(sensorBST()){
         BSTflag=1;
     }
     
     if(sensorBNT()){
         BNTflag=1;
     }
     
    if((BNTflag==1 ||BSTflag==1)||(BNTflag==1 && BSTflag==1)){
        Tflag=1;
    }
    if(sensorLS()){
        LSflag=1;
    }
     if(sensorPBN()){
         PBNflag=1;
     }
     if(sensorPBS()){
         PBSflag=1;
     }
     if(sensorPLS()){
         PLSflag=1;
     }
    if((PBNflag==1 || PBSflag==1 || PLSflag==1)||(PBNflag==1 && PBSflag==1)||(PLSflag==1&& PBSflag==1) || (PBNflag==1&& PLSflag==1)|| (PBNflag==1 && PBSflag==1 && PLSflag==1)){
        Pflag=1;
    }
}

ISR(TIMER0_COMPA_vect) {
    
    if(HazButton()){
        state=HS;
    }
    
   
    
    //1ms accuracy - used for time period of the traffic light system
    uint8_t value=0;
    value = AtoD;
    //value=read_AtoD();)
    
    
    millis++;
    
    if(state==HS){
        if(millis%1000==0){
            HSperiod++;
        }
    }
    
    if(value < 1){
        value = 1;
    }
    
    if(value > 240){
        value = 240;
    }
    
    if(value > 1 && value < 30){
    if (millis%1000==0){
        count++;
        period++;
        PORTC^=0x08;
}
}
     if(value > 29 && value < 60){
    if (millis%847==0){
        count++;
        period++;
        PORTC^=0x08;
}
}
     if(value > 59 && value < 90){
    if (millis%693==0){
        count++;
        period++;
        PORTC^=0x08;
}
}
     if(value > 89 && value < 120){
    if (millis%540==0){
        count++;
        period++;
        PORTC^=0x08;
}
}//NEED TO FIX THESE TIME PERIODS
     if(value > 119 && value < 150 ){
    if (millis%386==0){
        count++;
        period++;
        PORTC^=0x08;
}
}
     if(value > 149 && value < 180){
    if (millis%310==0){
        count++;
        period++;
        PORTC^=0x08;
}
}
     if(value > 179 && value < 210){
    if (millis%233==0){
        count++;
        period++;
        PORTC^=0x08;
}
}
     if(value > 209  && value < 240){
    if (millis%80==0){
        count++;
        period++;
        PORTC^=0x08;
}
}
    //SIREN BELOW
    if(state==Pflash){
        //PORTB|=_BV(PB1);
    TCNT1=7000; //3 seconds passed during pedestrian flash
    OCR1A++;
    PORTC^=0x04; //diagnostic light toggle
    if(OCR1A<7000){
        OCR1A=4000;
    }
    }
    if((state=P) && (count%100==0)){
        OCR1A=4000;
        PORTC^=0x04; //diagnostic light toggle
    }
        else if(count%5000==0) {
        OCR1A=4000;
        PORTC^=0x04; //diagnostic light toggle
       
    }
    }
    


   void LCD_TimePeriod(uint8_t addr){
    //might need to use % to put each char in respective spot
    //e.g. if TP = 11 use count%10=0 - counter2++, and place counter2 in the spot
    
    //LCD_Position(addr,0x0B);
    LCD_quad(addr, 0x0B,count);
} 
   
   
    


//HS, BB, BCgreen, BCyellow, BCred, BTgreen, BTyellow, BTred, LSgreen,LSyellow, LSred, P, T

int main(void) {
    /* Replace with your application code */
    setup_SPI();
    setup_AtoD();
    setup_I2C();
    setup_LCD(addr);
    
    if(HSperiod<10){
    state=HS;
    HazS();
    } 
    
    
    
    LCD_sensorBSM(addr);
    LCD_sensorBT(addr);
    LCD_sensorBST(addr);
    LCD_sensorBB(addr);
    LCD_sensorBNM(addr);
    LCD_sensorBNT(addr);
    LCD_sensorLS(addr);
    LCD_sensorPBN(addr);
    LCD_sensorPBS(addr);
    LCD_sensorPLS(addr);
    LCD_TimePeriod(addr);
    
    LCD_Potentiometer(addr);
    LCD_State(addr);
    
    timer_0setup();
    timer_1setup();
    
    
    
    
    
    
    

    
    
    
    while (1) {
        //if()
        switch (state){
          
    
            case HS:
                //if(sensorBB()){
                if(BBflag==1){
                    if(period<3){
                state=BB; //Broadway Bus
                /*BBLights(Blue);
                BCLights(Red,Green); //BSM Green
                BTLights(Red);
                LSLights(Red);
                PLights(PRed);
                TLights(White); //Tram can run concurrently*/
                BBus();
                BBflag=0;
                } 
                }
               
                break;
            case BB:
                period=0;
               
                    //if((period<10|| (period>10 && ((sensorBSM()||sensorBNM())||(sensorBSM()&&sensorBNM()))))){
                if(((period<10)|| (period>10 && ((BSMflag==1||BNMflag==1)||(BSMflag==1 && BNMflag==1))))){
                state=BCgreen; //Broadway Car Green
                //BBLights(Blue); BB lights off
                /*BCLights(Green,Green); //BSM & BNM Green
                BTLights(Red);
                LSLights(Red);
                PLights(PRed);
                TLights(White);*/
                BCg();
                }
                break;
            case BCgreen:
                period=0;
                if(period<3){
                    BSMflag=0;
                    BNMflag=0;
                state=BCyellow;//Broadway Car Yellow
                /*BBLights(Blue);
                BCLights(Yellow,Green); 
                BTLights(Red);
                LSLights(Red);
                PLights(PRed);
                TLights(White);*/
                BCy();
                }
                break;
            case BCyellow:
                period=0;
                if(period<3){
                state=BCred; //Broadway Car Red
                /*//BBLights(Blue);
                BCLights(Red,Green); 
                BTLights(Red);
                LSLights(Red);
                PLights(PRed);
                TLights(TRed);*/
                BCr();
                }
                break;
            case BCred:
                period=0;
                //if(sensorBT()){
                if(BTflag==1){
                if(period<2  || (period<5 && sensorBT())){
                state=BTgreen; //Broadway Turn Green
                
                /*//BBLights(Blue);
                BCLights(Red,Green); //or either BCLights(Green,Red);
                BTLights(Green);
                LSLights(Red);
                PLights(PRed);
                TLights(TRed);*/
                BTg();
                
                }
                }
                BTflag=2;
                break;
            case BTgreen:
                period=0;
                if(BTflag==2){
                if(period<3){
                state=BTyellow; //Broadway Turn Yellow
                 //BBLights(Blue);
                /*
                BCLights(Red,Yellow); //BSM Green
                BTLights(Yellow);
                LSLights(Red);
                PLights(PRed);
                TLights(TRed);*/
                BTy();
                }
                }
                break;
            case BTyellow:
                if(BTflag==2){
                if(period>3 && period<6){
                state=BTred; //Broadway Turn Red
                 /*BBLights(Blue);
                BCLights(Red,Red); //BSM Green
                BTLights(Red);
                LSLights(Red);
                PLights(PRed);
                TLights(TRed);*/
                BTr();
                }
                }
                BTflag=0;
                break;
            case BTred:
                period=0;
                //if(sensorLS()){
                if(LSflag==1){
                if((period<3 || (period<5 && sensorLS()))){
                state=LSgreen; //Little Street Green
                 //BBLights(Blue);
                /*BCLights(Red,Red); //BSM Green
                BTLights(Red);
                LSLights(Green);
                PLights(PRed);
                TLights(TRed);*/
                LSg();
                }
                }
                LSflag=2;
                break;
            case LSgreen:
                period=0; //reset period counter
                //if(state==LSgreen){
                if(LSflag==2){
                //if(period>3 && period<6){
                    if(period<3){
                state=LSyellow; //Little Street Yellow
                 //BBLights(Blue);
                /*BCLights(Red,Red); //BSM Green
                BTLights(Red);
                LSLights(Yellow);
                PLights(PRed);
                TLights(TRed);*/
                LSy();
                }
                }
                LSflag=3; 
               break;
            case LSyellow:
                period=0;
                if(LSflag==3){
                    //if(period>6 && period<9){
                    if(period<3){
                state=LSred; //Little Street Red
                 //BBLights(Blue);
                /*BCLights(Red,Red); //BSM Green
                BTLights(Red);
                LSLights(Red);
                PLights(PRed);
                TLights(TRed);*/
                LSr();
                }
        }
                LSflag=0;
                break;
            case LSred:
                //if(sensorPBN() || sensorPBS() || sensorPLS()){
                if(Pflag==1){
                state=P; //Pedestrian
                period=0;
                if(period<7){
                 //BBLights(Blue);
                /*BCLights(Red,Red); //BSM Green
                BTLights(Red);
                LSLights(Red);
                PLights(Green);
                TLights(TRed);*/
                    Ped();
                    Pflag=0;
                }
                }
                break;
            case P:
                period=0;
                if(period<3){
                state=Pflash; //Pedestrian Flash
                 //BBLights(Blue);
                /*BCLights(Red,Red); //BSM Green
                BTLights(Red);
                LSLights(Red);
                PLights(Green); //need to flash these lights
                TLights(TRed);*/
                PF();
                }
                break;
                
            case Pflash:
                //3 second time period where Pedestrian light is red
                //3 second transition between pedestrian and tram
                period=0;
                if(period<3){
                    
                    state=Pdont;
                    PD();
                }
                break;
            case Pdont:
                
                //if(((sensorBNT() || sensorBST()))||((sensorBNT()&&sensorBST()))){
                if(Tflag==1){
                if(period<5 ||(period<10 && (sensorBNT()||sensorBST()))){
                state=T; //Tram
                 //BBLights(Blue);
                /*BCLights(Red,Red); //BSM Green
                BTLights(Red);
                LSLights(Red);
                PLights(PRed);
                TLights(White);*/
                Tram();
                }
                }
                Tflag=0;
                break;

            case T:
                state=BB; //Broadway Bus
                period=0;
                if(BBflag==1){
                if(period<3){    
                /*BBLights(Blue);
                BCLights(Red,Red); //BSM Green
                BTLights(Red);
                LSLights(Red);
                PLights(PRed);
                TLights(Red);*/
                    BBus();
                }
                }
                break;
            case L:
                if(HazButton()){
                    state=HS;
                    HazS();
                }
                
            default:
                state=BCgreen;
                BCg();
                    
        }
    }
        
}






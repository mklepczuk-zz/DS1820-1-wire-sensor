#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 1000000UL  // 1 MHz
#define DS1820_PIN        PB5
#define output_low(port,pin)  port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define set_input(portdir,pin)  portdir &= ~(1<<pin)
#define set_output(portdir,pin) portdir |= (1<<pin)
#define TRUE 1
#define FALSE 0

/* -------------------------------------------------------------------------- */
/*                         Parametry czasowe DS1820 w [us]                    */
/* -------------------------------------------------------------------------- */

#define RESET             480     // czas potrzebny do wykonannia resetu
#define START             3       // opóźnienie przed odczytem
#define WAIT              70      // opóźnienie po sygnale resetu
#define READ_DELAY        10      // opóźnienie przy odczycie bitu
#define BYTEREAD_DELAY    120     // opóźnienie przy odczycie bajtu
#define BYTEWRITE_DELAY   105     // opóźnienie przy zapisie bajtu

/* -------------------------------------------------------------------------- */
/*                            Rejestry DS1820                                 */
/* -------------------------------------------------------------------------- */

#define TEMP_LSB      0
#define TEMP_MSB      1
#define COUNT_REMAIN  6
#define COUNT_PER_C   7
#define SCRATCH_LEN   9


/* -------------------------------------------------------------------------- */
/*                            Rozkazy                                         */
/* -------------------------------------------------------------------------- */

#define SEARCH_ROM     0xF0
#define READ_ROM       0x33
#define MATCH_ROM      0x55
#define SKIP_ROM       0xCC
#define ALARM_SEARCH   0xEC
#define CONVERT_TEMP   0x44
#define WRITE_SCRATCH  0x4E
#define READ_SCRATCH   0xBE
#define COPY_SCRATCH   0x48
#define RECALL         0xB8


#define DS18B20_FAMILY_CODE     0x28  //kod rodziny dla DS18B20
#define DS18S20_FAMILY_CODE     0x10  //kod rodziny dla DS18S20

/* -------------------------------------------------------------------------- */
/*                            Zmienne globalne                                */
/* -------------------------------------------------------------------------- */

typedef unsigned char  uint8;
typedef unsigned int uint16;
static unsigned char Last_difference_bit;
static unsigned char Number;

/*******************************************************************************
 * Funkcja:   InterruptsDisable
 * Cel:       Ustawienie flagi przerwań na false, przerwania nie są wykonywane.
 * Wejście:   -
 * Wyjście:   -
 * Zwraca:    -
 ******************************************************************************/
void InterruptsDisable()
{
    SREG &= 0x7F;
}

/*******************************************************************************
 * Funkcja:   InterruptsEnable
 * Cel:       Ustawienie flagi przerwań na true, odblokowanie przerwań.
 * Wejście:   -
 * Wyjście:   -
 * Zwraca:    -
 ******************************************************************************/
void InterruptsEnable()
{
    SREG |= 0x80;
}

/*******************************************************************************
 * Funkcja:   Reset
 * Cel:       Zresetowanie czujnika.
 * Wejście:   -
 * Wyjście:   -
 * Zwraca:    FALSE jeśli przynajmniej jedno urządzenie jest dołączone do magistrali, TRUE w innym przypadku
 ******************************************************************************/
bool Reset(void)
{
   bool bPresPulse;
   uint8 temp = 0;

   InterruptsDisable();
   
   // reset impulsu
   set_output(DDRB,DS1820_PIN);
   output_low(PORTB,DS1820_PIN);
   _delay_us(RESET);
   set_input(DDRB,DS1820_PIN);

   // czekaj dopóki magistrala 1-wire nie osiągnie stanu wysokiego
   _delay_us(WAIT);

   // wykryto obecność impulsu
   temp = PINB;
   temp >>= 5;
   if (temp)
   {
    bPresPulse = TRUE;
   }
   else
   {
    bPresPulse = FALSE;
   }

   InterruptsEnable();
   _delay_us(417);
   
   return bPresPulse;
}

/*******************************************************************************
 * Funkcja:   Read
 * Cel:       Odczyt pojedynczego bitu z czujnika DS1820.
 *
 * Wejście:   -
 * Wyjście:   -
 * Zwraca:    Wartośc boolowska bitu, który został odczytany
 ******************************************************************************/
bool Read(void)
{
   bool bBit;
   uint8 temp = 0;

   InterruptsDisable();

   set_output(DDRB,DS1820_PIN);
   output_low(PORTB,DS1820_PIN);
   _delay_us(START);
   set_input(DDRB,DS1820_PIN);
   _delay_us(READ_DELAY);

   temp = PINB;
   temp >>= 5;
   if (temp)
   {
    bBit = TRUE;
   }
   else
   {
    bBit = FALSE;
   }
   
   InterruptsEnable();

   return (bBit);
}

/*******************************************************************************
 * Funkcja:   Write
 * Cel:       Zapisuje pojedynczy bit do urządzenia DS1820
 * Wejście:   bBit        wartość bitu, która ma zostać wpisana
 * Wyjście:   -
 * Zwraca:    -
 ******************************************************************************/
void Write(bool bBit)
{
   InterruptsDisable();
   set_output(DDRB,DS1820_PIN);
   
   if (bBit & 1) {
      output_low(PORTB,DS1820_PIN);
      _delay_us(READ_DELAY);
      output_high(PORTB,DS1820_PIN);
      InterruptsEnable();
      _delay_us(55);
   }
   
   else {
      output_low(PORTB,DS1820_PIN);
      _delay_us(WAIT);
      output_high(PORTB,DS1820_PIN);
      InterruptsEnable();
      _delay_us(5);
   }
}

/*******************************************************************************
 * Funkcja:   ReadByte
 * Cel:       Odczyt całego bajtu z czujnika DS1820
 * Wejście:   -
 * Wyjście:   -
 * Zwraca:    uint8   Bajt odczytany z czujnika DS1820
 ******************************************************************************/
uint8 ReadByte(void)
{
   uint8 i;
   uint8 bit = 0;

   for (i=0 ; i < 8; i++)
   {
      if ( Read() )
      {
         bit |= (1 << i);
      }
      _delay_us(BYTEREAD_DELAY);
   }
   return(bit);
}

/*******************************************************************************
 * Funkcja:   WriteByte
 * Cel:       Zapisuje pojedynczy bajt w sensorze Ds1820
 * Wejście:   val_u8    bajt to wpisania
 * Wyjście:   -
 * Zwraca:    -
 ******************************************************************************/
void WriteByte(uint8 to_write)
{
   uint8 i;
   uint8 value;

   for (i=0; i < 8; i++)    // wpisywanie bajtu, bit po bicie
   {
      value = to_write >> i;  // przesuwa miejsce wpisu do pozycji "i"
      value &= 0x01;            // kopiuje wartość do temp
      Write(value);       // wpisz bit wykorzystując funkcję Write
   }

   //DelayMicro(BYTEWRITE_DELAY);
}


/* -------------------------------------------------------------------------- */
/*                             Interfejs API                                  */
/* -------------------------------------------------------------------------- */


/*******************************************************************************
 * Funkcja:   AddressOfDevice
 * Cel:       Adresuje pojedyncze lub wszystkie urządzenia dołączone do magistrali
 * Wejście:   AddressingType      używa MATCHROM do wyboru pojedynczego urządzenia
 *                                lub SKIPROM aby wybrać wszystkie
 * Wyjście:   -
 * Zwraca:    -
 ******************************************************************************/
void AddressOfDevice(uint8 AddressingType,uint8 *newAddr)
{
   uint8 i;
   
   if (AddressingType == MATCH_ROM)
   {
      WriteByte(MATCH_ROM);     // adres pojedynczego urządzenia
      for (i = 0; i < 8; i ++)
      {
         WriteByte(newAddr[i]);
      }
   }
   else
   {
      WriteByte(SKIP_ROM);     // adres wszystkich urządzeń
   }
}

/*******************************************************************************
 * Funkcja:   DeviceFind
 * Cel:       Znajduje następnę urządzenie na magistrali
 *
 * Wejście:   -
 * Wyjście:   Device_address[]  kod ROM następnego urządzenia
 * Zwraca:    bool              TRUE jeśli jest co najmniej jedno urządzenie na magistrali
 *                              FALSE w innym przypadku
 ******************************************************************************/
bool DeviceFind(uint8 *newAddr)
{
    uint8 address_index;
    uint8 mask = 1;
    uint8 position_of_bit = 1;
    bool value_of_bit;
    bool com_value_of_bit;
    uint8 search_bit;
    bool reset_status;
    bool device_presence = FALSE;
    uint8 Device_address[8];

    //inicjalizacja adresu ROM
    for (address_index = 0; address_index < 8; address_index ++)
    {
        Device_address[address_index] = 0x00;
    }

    address_index = 0;
    reset_status = Reset();         // zresetuj

    if (reset_status)     // nie znaleziono urządzenia
      return FALSE;

    // wyślij komendę search_ROM
    WriteByte(SEARCH_ROM);
    
    do
    {
        value_of_bit = Read();
        _delay_us(120);
        com_value_of_bit = Read();
        if( (value_of_bit == 1) && (com_value_of_bit == 1)){
          break;
        }
        else
        {
            // urządzenia mają taką samą wartość logiczną na bicie
            if (value_of_bit != com_value_of_bit)
            {
                // otrzymaj wartość bitu
                search_bit = value_of_bit;
            }
            // występuje konfikt w biezącym kodzie ROM
            else
            {
                Last_difference_bit = 1;
                search_bit = 1;
                if(Number == 2)
                  search_bit = 0;
                Number = 2; 
            }
           _delay_us(120);
           // zapamiętaj bit w pamięci ROM
           if (search_bit == 1)
           {
               Device_address[address_index] |= mask;
           }
           else
           {
               Device_address[address_index] &= ~mask;
           }

           Write(search_bit);

           // inkrementacja pozycji bitu
           position_of_bit++;

           // przesunięcie maski
           mask = mask << 1;

           // Sprawdź czy bajt został zakończony 
           if (mask == 0)
           {
               address_index++;  // przesuń do kolejnej maski ROM
               mask = 1;      // zaktualizuj maskę
           }
        }
        
    } while (address_index < 8);
    
    // jeśli wyszukiwanie nie powiodło się
    if (!(position_of_bit < 65))
    {
        // Szukanie zakończone powodzeniem
        device_presence = TRUE;
        for (uint8 i = 0; i < 8; i++) newAddr[i] = Device_address[i];
    }
    return device_presence;
  }

/*******************************************************************************
 * Funkcja:   TemperatureRead
 * Cel:       Pobierz nieprzetworzoną wartość odczytaną z czujnika.
 *
 *              Opis pamięci Scratchpad
 *              Byte  Register
 *              0     Temperature_LSB
 *              1     Temperature_MSB
 *              2     Temp Alarm High / User Byte 1
 *              3     Temp Alarm Low / User Byte 2
 *              4     Reserved
 *              5     Reserved
 *              6     Count_Remain
 *              7     Count_per_C
 *              8     CRC
 *
 *              Obliczenie temperatury dla DS18S20 (Family Code 0x10):
 *              =======================================================
 *                                             (Count_per_C - Count_Remain)
 *              Temperature = temp_raw - 0.25 + ----------------------------
 *                                                     Count_per_C
 *
 *              Gdzie temp_raw jest wartością z temp_MSB i temp_LSB
 *              z usuniętym najmniej znaczącym bitem (the 0.5C bit).
 *
 *
 *              Obliczenie temperatury dla DS18B20 (Family Code 0x28):
 *              =======================================================
 *                        bit7   bit6   bit5   bit4   bit3   bit2   bit1   bit0
 *              LSB       2^3    2^2    2^1    2^0    2^-1   2^-2   2^-3   2^-4
 *                        bit15  bit14  bit13  bit12  bit3   bit2   bit1   bit0
 *              MSB       S      S      S      S      S      2^6    2^5    2^4
 *
 *              Wartość temperatury jest przechowywana jako 16-bit wartość
 *        w rejestrze temperatury z bitami znaku. Bity znaku (S)
 *        wskazują czy temperatura jest dodatnia czy ujemna:
 *        dla dodatnich S = 0 a dla ujemnych S = 1.
 *
 * Zwraca:     signed int         Nieprzetworzona wartość w rozdzielczości 1/256°C
 ******************************************************************************/
signed int TemperatureRead(uint8 *newAddr)
{
    uint8 i;
    uint16 value_of_temp;
    uint16 highresolution;
    uint8 scratchpad[SCRATCH_LEN];

  
    /* --- rozpocznij konwersję temperatury -------------------------------------- */
    Reset();
    AddressOfDevice(MATCH_ROM, newAddr); // adres urządzenia
    WriteByte(CONVERT_TEMP);    // początek konwersji
    _delay_ms(750);         // czekaj na koniec konwersji
  


    /* --- odczyt pamięci Scratchpad ---------------------------------------------------- */
    Reset();
    AddressOfDevice(MATCH_ROM, newAddr); // adres urządzenia
    WriteByte(READ_SCRATCH);    // odczyt pamięci Scratchpad  

    // odczytaj dane z pamięci Scratchpad
    for (i=0; i < SCRATCH_LEN; i++)
    {
        scratchpad[i] = ReadByte();
    }

    /* --- Oszacowanie temperatury --------------------------------------------- */
    /* Formuła do odczytu temperatury:
     Temp = Temp_read - 0.25 + ((Count_per_C - Count_Remain)/Count_per_C) */

    // nieprzetworzona wartość temperatury (rozdzielczość 0.5°C)
    value_of_temp = 0;
    value_of_temp = (uint16)((uint16)scratchpad[TEMP_MSB] << 8);
    value_of_temp |= (uint16)(scratchpad[TEMP_LSB]);

    if (newAddr[0] == DS18S20_FAMILY_CODE)
    {
        // odczyt temperatury w rozdzielczości 1°C
        value_of_temp >>= 1;
    
        // rozdzielczość temperatury to RESOLUTION (0x100), więc 1°C odpowiada 0x100
        // przetwórz temperaturę do rozdzielczości 1/256°C
        value_of_temp = ((uint16)value_of_temp << 8);
    
        // odejmij 0.25°C
        value_of_temp -= ((uint16)0x100 >> 2);
    
        // oblicz wysoką rodzielczość
        highresolution = scratchpad[COUNT_PER_C] - scratchpad[COUNT_REMAIN];
        highresolution = ((uint16)highresolution << 8);
        if (scratchpad[COUNT_PER_C])
        {
            highresolution = highresolution / (uint16)scratchpad[COUNT_PER_C];
        }
    
        // oblicz końcowy wynik
        highresolution = highresolution + value_of_temp;
    }
    else
    {
        // 12 bitowa wartość temperatury ma rodzielczość 0.0625°C
        // przesunięcie w lewo o 4 pozycje, aby zyskać rozdzielczość 1/256°C
        highresolution = value_of_temp;
        highresolution <<= 4;
    }
    return (highresolution);
}
//***************************************************

#include <LiquidCrystal.h>;
LiquidCrystal lcd(2,3,4,5,6,7);

int main()
{
  lcd.begin(16,2);
  
  Serial.begin(9600);
  uint8 address[8], address2[8];
  signed int temperature;
  uint8 temperature_msb;
  uint8 temperature_lsb;

  DeviceFind(address);
  DeviceFind(address2);

  Serial.println(address[1],HEX); _delay_ms(3);
  Serial.println(address2[1],HEX); _delay_ms(3);

  for(byte i = 0; i < 10; i++){
  temperature = TemperatureRead(address);
  temperature_lsb = temperature;
  temperature_msb = temperature >> 8;
  Serial.println("Pierwszy czujnik");
  Serial.print(temperature_msb); _delay_ms(3);
  Serial.print(",");
  Serial.print(temperature_lsb); _delay_ms(3);
  Serial.println(" C");
  
  lcd.setCursor(0,0);
  lcd.print("Pierwszy czujnik:");
  lcd.setCursor(0,1);
  lcd.print(temperature_msb);
  lcd.setCursor(2,1);
  lcd.print(",");
  lcd.setCursor(3,1);
  lcd.print(temperature_lsb);
  lcd.setCursor(6,1);
  lcd.print(" C");
  _delay_ms(2000);
  lcd.clear();
  
  temperature = TemperatureRead(address2);
  temperature_lsb = temperature;
  temperature_msb = temperature >> 8;
  Serial.println("Drugi czujnik");
  Serial.print(temperature_msb); _delay_ms(3);
  Serial.print(",");
  Serial.print(temperature_lsb); _delay_ms(3);
  Serial.println(" C");
  
  lcd.setCursor(0,0);
  lcd.print("Drugi czujnik:");
  lcd.setCursor(0,1);
  lcd.print(temperature_msb);
  lcd.setCursor(2,1);
  lcd.print(",");
  lcd.setCursor(3,1);
  lcd.print(temperature_lsb);
  lcd.setCursor(6,1);
  lcd.print(" C");
  _delay_ms(2000);
  lcd.clear();
}

return 0;
}

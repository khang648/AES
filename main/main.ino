#include <RA8875.h>
#include <Wire.h>
#include <SPI.h>
#include <avr/interrupt.h>
#include <EEPROM.h>

#define RA8875_INT             4
#define RA8875_CS              10 
#define RA8875_RESET           9

#define DISPLAY_WIDTH          480
#define DISPLAY_HEIGHT         272

#define BACK_GROUND_COLOR           0x0803
#define WINDOW_COLOR                0xb6b6
#define FRAME_COLOR                 0x042b
#define TEXT_COLOR_1                0xffff
#define BUTTON_TEXT_COLOR           0x0000
#define BUTTON_COLOR_1              0x6767
#define COMPLETE_BACKGROUND_COLOR   0xd6d6
#define COMPLETE_CIRCLE_COLOR       0x2727
#define PUMP_CIRCLE_COLOR           0x2727
#define WAIT_CIRCLE_COLOR           0x2727


#define TEXT_SIZE_LARGE        2
#define TEXT_SIZE_MEDIUM       1
#define TEXT_SIZE_SMALL        0

#define X_STEP_PIN       16
#define X_ENABLE_PIN     14
#define X_DIR_PIN        18
#define X_SW_PIN         25
#define Y_STEP_PIN       17
#define Y_ENABLE_PIN     15
#define Y_DIR_PIN        12
#define Y_SW_PIN         27
#define Z_STEP_PIN       23
#define Z_ENABLE_PIN     21
#define Z_DIR_PIN        19 
#define Z_SW_PIN         29

#define O1_PIN           46
#define O2_PIN           44
#define O3_PIN           38
#define O4_PIN           36
#define O5_PIN           A14
#define O6_PIN           A12
#define O7_PIN           A6
#define O8_PIN           A4
#define O9_PIN           A2
#define O10_PIN          A0
#define O11_PIN          A1
#define O12_PIN          A3
#define PUMP_PIN         A7
#define PRESSURE_PIN     A5
#define PWM_PIN          2

#define wait_time        100

uint16_t volatile tx, ty, x, y;
boolean newprogram1=1, newprogram2=0, newprogram3=0;
uint8_t volatile stages=8, group_stage = 1, pass_group=0;   //stages: Biến chứa giá trị số stage 
uint8_t keyboard[] = {7,8,9,4,5,6,1,2,3,0}; 
uint8_t volatile screen = 2;   //Chuỗi xác định trạng thái active của các Screen 
char u1[20], u2[20], u3[20], u4[20], u_sample[20];   //Lần lượt chứa giá trị dạng string của Volume, Low, Off, High (Dùng chung cho tất cả các stage)
uint32_t key[5];   //Lần lượt chứa giá trị nhập từ bàn phím cho Volume, Low, Off, High, Samples (Dùng chung cho tất cả các stage)
uint8_t n;   //Biến trạng thái để xác định giữa Volume, Low, Off, High giá trị nào đang được chọn để nhập giá trị từ bàn phím (Dùng chung cho tất cả các stage)
uint8_t stage_config;   //Biến trạng thái để xác định sẽ lưu giá trị cho stage nào
uint16_t volume[7], vacuum_low[7], vacuum_off[7], vacuum_high[7];
int m[7],s[7];
uint8_t samples = 96;
uint8_t volatile chemical[7];
uint16_t volatile count0=0, count1=0, count2=0, count=0;
uint8_t volatile run_time=0, run_scr=0;
uint8_t x_limit=0,y_limit=0;
uint8_t volatile home_value=0,run_value=0;
uint8_t volatile putchemin=0, completed=0, firstmove=0;
uint8_t count_delay=0, delayPress=0;
uint8_t pause_click=0;
uint8_t vaccume_state = 0;
uint8_t pulse_high_set = 255;
uint8_t pulse_low_set = 150;
uint8_t ispress=0;
bool enablepress=1;
bool run_switch_screen = 0;

RA8875 tft = RA8875(RA8875_CS,RA8875_RESET);

void setup()
{   
    pinMode(X_ENABLE_PIN, OUTPUT);
    pinMode(Y_ENABLE_PIN, OUTPUT);
    pinMode(Z_ENABLE_PIN, OUTPUT);
    pinMode(X_STEP_PIN, OUTPUT);
    pinMode(X_DIR_PIN, OUTPUT);
    pinMode(Y_STEP_PIN,OUTPUT);
    pinMode(Y_DIR_PIN, OUTPUT);
    pinMode(Z_STEP_PIN, OUTPUT);
    pinMode(Z_DIR_PIN, OUTPUT);
    pinMode(X_SW_PIN,INPUT_PULLUP);
    pinMode(Y_SW_PIN,INPUT_PULLUP);
    pinMode(Z_SW_PIN,INPUT_PULLUP);
    pinMode(PUMP_PIN, OUTPUT);
    pinMode(PRESSURE_PIN, OUTPUT);
    pinMode(PWM_PIN,OUTPUT);
    pinMode(O1_PIN, OUTPUT);
    pinMode(O2_PIN, OUTPUT);
    pinMode(O3_PIN, OUTPUT);
    pinMode(O4_PIN, OUTPUT);
    pinMode(O5_PIN, OUTPUT);
    pinMode(O6_PIN, OUTPUT);
    pinMode(O7_PIN, OUTPUT);
    pinMode(O8_PIN, OUTPUT);
    pinMode(O9_PIN, OUTPUT);
    pinMode(O10_PIN, OUTPUT);
    pinMode(O11_PIN, OUTPUT);
    pinMode(O12_PIN, OUTPUT);
  
    digitalWrite(X_ENABLE_PIN, HIGH);
    digitalWrite(Y_ENABLE_PIN, HIGH);
    //digitalWrite(Z_ENABLE_PIN, HIGH);
    digitalWrite(O1_PIN,LOW);
    digitalWrite(O2_PIN,LOW);
    digitalWrite(O3_PIN,LOW);
    digitalWrite(O4_PIN,LOW);
    digitalWrite(O5_PIN,LOW);
    digitalWrite(O6_PIN,LOW);
    digitalWrite(O7_PIN,LOW);
    digitalWrite(O8_PIN,LOW);
    digitalWrite(O9_PIN,LOW);
    digitalWrite(O10_PIN,LOW);
    digitalWrite(O11_PIN,LOW);
    digitalWrite(O12_PIN,LOW);  
    digitalWrite(PRESSURE_PIN,LOW);
    analogWrite(PWM_PIN,0);

    Wire.begin();

    tft.begin(RA8875_480x272);
    tft.touchBegin(RA8875_INT);

    cli();     
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 = 0;  
    TCCR1B |= (1 << CS11) | (1 << CS10) ;   
    TCNT1 = 40536;
    TIMSK1 = (1 << TOIE1);

    // TCCR2A = 0;
    // TCCR2B = 0;
    // TIMSK2 = 0;      
    // TCCR2B |= (1 << CS21) | (1 << CS20) | (1 << CS22) ;   
    // TCNT2 = 100;
    // TIMSK2 = (1 << TOIE2);
    sei();

    programScreen();
}

ISR (TIMER1_OVF_vect) 
{
    count1++;
    count2++;

    if(ispress==1)
    {
        enablepress = 0;
        count0++;
        if(count0>=7)
        {
            ispress=0;
            enablepress = 1;
            count0=0;
        }
    }

    if(count2>=3)
    {       
        pressButton();
        count2 = 0;
    }

    if(count1==10)
    {
        count ++;
        if(screen==5)
        {
            for(uint8_t tg1=0; tg1<4; tg1++)
            {
                if(run_time==tg1+1)
                {
                    s[tg1]--;
                    if(m[tg1]>0 && s[tg1]<0)
                    {
                        m[tg1]--;
                        s[tg1]=59;
                    }
                    if(m[tg1]==0 && s[tg1]<0)
                    { 
                        s[tg1]=0;
                    }
                    if(group_stage==1)
                    {
                        tft.fillRect(tg1*((DISPLAY_WIDTH/4)-10)+2,35+29,(DISPLAY_WIDTH/4)-14,DISPLAY_HEIGHT-66-33,BACK_GROUND_COLOR);
                        tft.drawCircle(tg1*((DISPLAY_WIDTH/4)-10)+55,177,40,0xe0e0);
                        tft.drawCircle(tg1*((DISPLAY_WIDTH/4)-10)+55,177,41,0xe0e0);
                        tft.drawCircle(tg1*((DISPLAY_WIDTH/4)-10)+55,177,42,0xe0e0);
                        tft.setFontScale(TEXT_SIZE_SMALL);
                        tft.setTextColor(0xe0e0);
                        tft.setCursor(tg1*((DISPLAY_WIDTH/4)-10)+19, 169);
                        tft.println("VACUUMING");

                        tft.setFontScale(TEXT_SIZE_MEDIUM);
                        tft.setTextColor(0xe0e0);
                        if(m[tg1]<10)
                        {  
                            tft.setCursor(tg1*((DISPLAY_WIDTH/4)-10)+18, 76);
                            tft.println(0);
                            tft.setCursor(tg1*((DISPLAY_WIDTH/4)-10)+33, 76);
                            tft.println(m[tg1]);
                        }
                        else
                        {
                            tft.setCursor(tg1*((DISPLAY_WIDTH/4)-10)+18, 76);
                            tft.println(m[tg1]);
                        }

                        tft.setCursor(tg1*((DISPLAY_WIDTH/4)-10)+48, 76);
                        tft.println(":");

                        if(s[tg1]<10)
                        {
                            tft.setCursor(tg1*((DISPLAY_WIDTH/4)-10)+63, 76);
                            tft.println(0);
                            tft.setCursor(tg1*((DISPLAY_WIDTH/4)-10)+78, 76);
                            tft.println(s[tg1]);
                        }
                        else
                        {
                            tft.setCursor(tg1*((DISPLAY_WIDTH/4)-10)+63, 76);
                            tft.println(s[tg1]);
                        }
                    }
                }
            }
            
            for(int tg2=0; tg2<3; tg2++)
            {   
                if(run_time==tg2+5)
                {
                    s[tg2+4]--;
                    if(m[tg2+4]>0 && s[tg2+4]<0)
                    {
                        m[tg2+4]--;
                        s[tg2+4]=59;
                    }
                    if(m[tg2+4]==0 && s[tg2+4]<0)
                    { 
                        s[tg2+4]=0;
                    }
                    if(group_stage==2)
                    {
                        tft.fillRect(tg2*((DISPLAY_WIDTH/4)-10)+2+39,35+29,(DISPLAY_WIDTH/4)-14,DISPLAY_HEIGHT-66-33,BACK_GROUND_COLOR);
                        tft.drawCircle(tg2*((DISPLAY_WIDTH/4)-10)+55+39,177,40,0xe0e0);
                        tft.drawCircle(tg2*((DISPLAY_WIDTH/4)-10)+55+39,177,41,0xe0e0);
                        tft.drawCircle(tg2*((DISPLAY_WIDTH/4)-10)+55+39,177,42,0xe0e0);
                        tft.setFontScale(TEXT_SIZE_SMALL);
                        tft.setTextColor(0xe0e0);
                        tft.setCursor(tg2*((DISPLAY_WIDTH/4)-10)+19+39, 169);
                        tft.println("VACUUMING");

                        tft.setFontScale(TEXT_SIZE_MEDIUM);
                        tft.setTextColor(0xe0e0);
                        if(m[tg2+4]<10)
                        {  
                            tft.setCursor(tg2*((DISPLAY_WIDTH/4)-10)+18+39, 76);
                            tft.println(0);
                            tft.setCursor(tg2*((DISPLAY_WIDTH/4)-10)+33+39, 76);
                            tft.println(m[tg2+4]);
                        }
                        else
                        {
                            tft.setCursor(tg2*((DISPLAY_WIDTH/4)-10)+18+39, 76);
                            tft.println(m[tg2+4]);
                        }

                        tft.setCursor(tg2*((DISPLAY_WIDTH/4)-10)+48+39, 76);
                        tft.println(":");

                        if(s[tg2+4]<10)
                        {
                            tft.setCursor(tg2*((DISPLAY_WIDTH/4)-10)+63+39, 76);
                            tft.println(0);
                            tft.setCursor(tg2*((DISPLAY_WIDTH/4)-10)+78+39, 76);
                            tft.println(s[tg2+4]);
                        }
                        else
                        {
                            tft.setCursor(tg2*((DISPLAY_WIDTH/4)-10)+63+39, 76);
                            tft.println(s[tg2+4]);
                        }
                    }
                }
            }
        }
        count1=0;
        //runScreenUpdate(run_scr);
    } 
    TCNT1 = 40536;
}

// ISR (TIMER2_OVF_vect) 
// {
//     count2++;
//     if(count2==3)
//     {
//         pressButton();
//         count2 = 0;
//     }
//     TCNT2 = 100;
// }

void loop()
{
    if(run_value==1)
    {
        run(); 
    }
}

void(* resetFunc) (void) = 0;

//////////////////// DISPLAY - START ////////////////////
void mainScreen()
{
    screen = 0;
    tft.fillScreen(BACK_GROUND_COLOR);
    tft.drawRect(0,0,DISPLAY_WIDTH-1,30,FRAME_COLOR);
    tft.fillRect(3,3,DISPLAY_WIDTH-7,30-6,FRAME_COLOR);
    tft.drawRect(0,DISPLAY_HEIGHT-30,DISPLAY_WIDTH-1,29,FRAME_COLOR);

    for(int i=0; i<4;i++)
    {   
        tft.drawRect(i*((DISPLAY_WIDTH/4)+1),37,(DISPLAY_WIDTH/4)-4,DISPLAY_HEIGHT-37*2,FRAME_COLOR);
        // tft.drawLine(i*((DISPLAY_WIDTH/4)+1),75,i*((DISPLAY_WIDTH/4)+1)+115,75,FRAME_COLOR);
        // tft.drawLine(i*((DISPLAY_WIDTH/4)+1),76,i*((DISPLAY_WIDTH/4)+1)+115,76,FRAME_COLOR);
        // tft.drawLine(i*((DISPLAY_WIDTH/4)+1),194,i*((DISPLAY_WIDTH/4)+1)+115,194,FRAME_COLOR);
        // tft.drawLine(i*((DISPLAY_WIDTH/4)+1),195,i*((DISPLAY_WIDTH/4)+1)+115,195,FRAME_COLOR);
    }

    tft.drawRoundRect(5,119,106,30,5,0x4646);
    tft.drawRoundRect(126,119,106,30,5,0xe7e7);
    tft.drawRoundRect(247,119,106,30,5,0x1818);
    tft.drawRoundRect(368,119,106,30,5,0xe0e0);

    tft.setTextColor(TEXT_COLOR_1);
    tft.setFontScale(TEXT_SIZE_SMALL);
    tft.setCursor(132,7);
    tft.println("AUTOMATIC EXTRACTION SYSTEM");

    tft.setCursor(14,126);
    tft.println("NEW PROGRAM");

    tft.setCursor(151,126);
    tft.println("HISTORY");

    tft.setCursor(277,126);
    tft.println("MANUAL");

    tft.setCursor(394,126);
    tft.println("SETTING");

    tft.setTextColor(TEXT_COLOR_1);
    tft.setFontScale(TEXT_SIZE_SMALL);
    tft.setCursor(345,248);
    tft.println("PHUSABIOCHEM.COM");
}

void programScreen()
{
    screen = 1;
    tft.fillScreen(BACK_GROUND_COLOR);
    tft.drawRect(0,0,DISPLAY_WIDTH-1,30,FRAME_COLOR);
    tft.fillRect(3,3,DISPLAY_WIDTH-7,30-6,FRAME_COLOR);
    tft.drawRect(0,DISPLAY_HEIGHT-30,DISPLAY_WIDTH-1,29,FRAME_COLOR);
    
    tft.setTextColor(TEXT_COLOR_1);
    tft.setFontScale(TEXT_SIZE_SMALL);
    tft.setCursor(197,7);
    tft.println("NEW PROGRAM");

    for(int i=0; i<3;i++)
    {   
        tft.drawRect(i*((DISPLAY_WIDTH/3)+1),37,(DISPLAY_WIDTH/3)-4,DISPLAY_HEIGHT-37*2,FRAME_COLOR);
        //tft.drawRoundRect(i*((DISPLAY_WIDTH/3)+4),37,(DISPLAY_WIDTH/3)-4,30,5,FRAME_COLOR);       

        // tft.drawCircle((i*(DISPLAY_WIDTH/3)+79),135,50,FRAME_COLOR);
        // tft.drawCircle((i*(DISPLAY_WIDTH/3)+79),135,51,FRAME_COLOR);
        //tft.drawCircle((i*(DISPLAY_WIDTH/3)+79),135,43,FRAME_COLOR);
        tft.drawCircle((i*(DISPLAY_WIDTH/3)+79),135,50,FRAME_COLOR);
        tft.drawCircle((i*(DISPLAY_WIDTH/3)+79),135,51,FRAME_COLOR);
        tft.drawCircle((i*(DISPLAY_WIDTH/3)+79),135,52,FRAME_COLOR);
    }

    tft.setTextColor(TEXT_COLOR_1);
    tft.setCursor(60,127);
    tft.println("ezMOP");
    
    tft.setCursor(226,120);
    tft.println("RNA");
    tft.setCursor(199,134);
    tft.println("Extraction");
    
    tft.setCursor(375,127);
    tft.println("Custom");

    tft.fillRoundRect(2,DISPLAY_HEIGHT-30+2,75,25,5,BUTTON_COLOR_1);
    tft.fillRoundRect(402,DISPLAY_HEIGHT-30+2,75,25,5,BUTTON_COLOR_1);
    tft.setTextColor(BUTTON_TEXT_COLOR);
    tft.setFontScale(TEXT_SIZE_SMALL);

    tft.setCursor(25, DISPLAY_HEIGHT-30+7);
    tft.println("BACK");
    tft.setCursor(425, DISPLAY_HEIGHT-30+7);
    tft.println("NEXT");
    // tft.setCursor(197,7);
    // tft.println("NEW PROGRAM");

    if(newprogram1==1)
    {
        tft.drawRect(1,38,154,196,0xa7a7);
        tft.drawRect(2,39,152,194,0xa7a7);
        tft.drawRect(3,40,150,192,0xa7a7);
        tft.drawRect(4,41,148,190,0xa7a7);

        tft.drawRect(162,38,154,196,BACK_GROUND_COLOR);
        tft.drawRect(163,39,152,194,BACK_GROUND_COLOR);
        tft.drawRect(164,40,150,192,BACK_GROUND_COLOR);
        tft.drawRect(165,41,148,190,BACK_GROUND_COLOR);

        tft.drawRect(323,38,154,196,BACK_GROUND_COLOR);
        tft.drawRect(324,39,152,194,BACK_GROUND_COLOR);
        tft.drawRect(325,40,150,192,BACK_GROUND_COLOR);
        tft.drawRect(326,41,148,190,BACK_GROUND_COLOR);

        tft.drawCircle((0*(DISPLAY_WIDTH/3)+79),135,50,0xa7a7);
        tft.drawCircle((0*(DISPLAY_WIDTH/3)+79),135,51,0xa7a7);
        tft.drawCircle((0*(DISPLAY_WIDTH/3)+79),135,52,0xa7a7);
    }
    if(newprogram2==1)
    {
        tft.drawRect(1,38,154,196,BACK_GROUND_COLOR);
        tft.drawRect(2,39,152,194,BACK_GROUND_COLOR);
        tft.drawRect(3,40,150,192,BACK_GROUND_COLOR);
        tft.drawRect(4,41,148,190,BACK_GROUND_COLOR);

        tft.drawRect(162,38,154,196,0xa7a7);
        tft.drawRect(163,39,152,194,0xa7a7);
        tft.drawRect(164,40,150,192,0xa7a7);
        tft.drawRect(165,41,148,190,0xa7a7);

        tft.drawRect(323,38,154,196,BACK_GROUND_COLOR);
        tft.drawRect(324,39,152,194,BACK_GROUND_COLOR);
        tft.drawRect(325,40,150,192,BACK_GROUND_COLOR);
        tft.drawRect(326,41,148,190,BACK_GROUND_COLOR);

        tft.drawCircle((1*(DISPLAY_WIDTH/3)+79),135,50,0xa7a7);
        tft.drawCircle((1*(DISPLAY_WIDTH/3)+79),135,51,0xa7a7);
        tft.drawCircle((1*(DISPLAY_WIDTH/3)+79),135,52,0xa7a7);
    }
    if(newprogram3==1)
    {
        tft.drawRect(1,38,154,196,BACK_GROUND_COLOR);
        tft.drawRect(2,39,152,194,BACK_GROUND_COLOR);
        tft.drawRect(3,40,150,192,BACK_GROUND_COLOR);
        tft.drawRect(4,41,148,190,BACK_GROUND_COLOR);

        tft.drawRect(162,38,154,196,BACK_GROUND_COLOR);
        tft.drawRect(163,39,152,194,BACK_GROUND_COLOR);
        tft.drawRect(164,40,150,192,BACK_GROUND_COLOR);
        tft.drawRect(165,41,148,190,BACK_GROUND_COLOR);

        tft.drawRect(323,38,154,196,0xa7a7);
        tft.drawRect(324,39,152,194,0xa7a7);
        tft.drawRect(325,40,150,192,0xa7a7);
        tft.drawRect(326,41,148,190,0xa7a7);

        tft.drawCircle((2*(DISPLAY_WIDTH/3)+79),135,50,0xa7a7);
        tft.drawCircle((2*(DISPLAY_WIDTH/3)+79),135,51,0xa7a7);
        tft.drawCircle((2*(DISPLAY_WIDTH/3)+79),135,52,0xa7a7);
    }
}

void configScreen()
{   
    group_stage = 1;
    screen = 2;

    tft.fillScreen(BACK_GROUND_COLOR);
    tft.drawRect(0,0,DISPLAY_WIDTH-1,30,FRAME_COLOR);
    tft.fillRect(3,3,DISPLAY_WIDTH-7,30-6,FRAME_COLOR);
    tft.drawRect(0,DISPLAY_HEIGHT-30,DISPLAY_WIDTH-1,29,FRAME_COLOR);

    tft.setTextColor(TEXT_COLOR_1);
    tft.setFontScale(TEXT_SIZE_SMALL);
    tft.setCursor(185,7);
    tft.println("CONFIGURATION");

    for(int i=0; i<4;i++)
    {   
        tft.drawRect(i*((DISPLAY_WIDTH/4)-10),60,(DISPLAY_WIDTH/4)-10,DISPLAY_HEIGHT-98,FRAME_COLOR);
        tft.drawRect(i*((DISPLAY_WIDTH/4)-10),60,(DISPLAY_WIDTH/4)-10,27,FRAME_COLOR);
    }

    for(int i=0; i<stages; i++)
    {    
        if(i==4)    break;
        tft.setTextColor(TEXT_COLOR_1);
        tft.setFontScale(TEXT_SIZE_SMALL);
        // tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+28, 64);
        // tft.println("STAGE");
        // tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+73, 64);
        // tft.println(i+1);

        tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+5, 105);
        tft.println("Volume");
        tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+5, 145);
        tft.println("Vacuum");
        tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+68, 125);
        tft.println("L");
        tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+68, 145);
        tft.println("O");
        tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+68, 165);
        tft.println("H");

        tft.setTextColor(0xe0e0);
        tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+83, 105);
        tft.println(volume[i]);
        tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+83, 125);
        tft.println(vacuum_low[i]);
        tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+83, 145);
        tft.println(vacuum_off[i]);
        tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+83, 165);
        tft.println(vacuum_high[i]);

        tft.drawLine(i*((DISPLAY_WIDTH/4)-10)+60,134,i*((DISPLAY_WIDTH/4)-10)+60,174,FRAME_COLOR);
        tft.drawLine(i*((DISPLAY_WIDTH/4)-10)+60,134,i*((DISPLAY_WIDTH/4)-10)+64,134,FRAME_COLOR);
        tft.drawLine(i*((DISPLAY_WIDTH/4)-10)+60,154,i*((DISPLAY_WIDTH/4)-10)+64,154,FRAME_COLOR);
        tft.drawLine(i*((DISPLAY_WIDTH/4)-10)+60,174,i*((DISPLAY_WIDTH/4)-10)+64,174,FRAME_COLOR);

        tft.fillRoundRect(i*((DISPLAY_WIDTH/4)-10)+30,200,50,25,5,BUTTON_COLOR_1);
        tft.setTextColor(BUTTON_TEXT_COLOR);
        tft.setFontScale(TEXT_SIZE_SMALL);
        tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+40, 204);
        tft.println("EDIT");
    }

    tft.fillRect((4*((DISPLAY_WIDTH/4)-10))+2,60,37,DISPLAY_HEIGHT-98,FRAME_COLOR);
    tft.fillTriangle(451,135,451,165,471,150,0xffff);

    if(newprogram1==1)
    {
        tft.setTextColor(TEXT_COLOR_1);
        tft.setFontScale(TEXT_SIZE_SMALL);
        tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+20, 65);
        tft.println("Priming 1");
        tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+20, 65);
        tft.println("Priming 2");
        tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+22, 65);
        tft.println("Load OBS");
        tft.setCursor(3*((DISPLAY_WIDTH/4)-10)+31, 65);
        tft.println("Wash 1");
    }

    if(newprogram2==1)
    {
        tft.setTextColor(TEXT_COLOR_1);
        tft.setFontScale(TEXT_SIZE_SMALL);
        tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+15, 65);
        tft.println("Activation");
        tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+20, 65);
        tft.println("Washing 1");
        tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+30, 65);
        tft.println("Sample");
        tft.setCursor(3*((DISPLAY_WIDTH/4)-10)+20, 65);
        tft.println("Washing 2");
    }

    tft.setCursor(5,37);
    tft.println("NUMBER OF SAMPLES:");
    tft.drawRect(157,34,45,21,BUTTON_COLOR_1);
    tft.drawRect(156,33,47,23,BUTTON_COLOR_1);
    tft.setTextColor(0xe0e0,0xffff);
    tft.setCursor(160,37);    
    sprintf(u_sample,"%5.0d",samples);    
    tft.print(u_sample);
    //tft.setCursor(160,37);    
    //tft.print(samples); 

    tft.fillRoundRect(2,DISPLAY_HEIGHT-30+2,75,25,5,BUTTON_COLOR_1);
    tft.fillRoundRect(402,DISPLAY_HEIGHT-30+2,75,25,5,BUTTON_COLOR_1);
    tft.fillRoundRect(201,DISPLAY_HEIGHT-30+2,75,25,5,BUTTON_COLOR_1);
    tft.fillRoundRect(402,33,75,25,5,BUTTON_COLOR_1);
    tft.setTextColor(BUTTON_TEXT_COLOR);
    tft.setFontScale(TEXT_SIZE_SMALL);
    tft.setCursor(25, DISPLAY_HEIGHT-30+7);
    tft.println("BACK");
    tft.setCursor(429, DISPLAY_HEIGHT-30+7);
    tft.println("RUN");
    tft.setCursor(224, DISPLAY_HEIGHT-30+7);
    tft.println("SAVE");
    tft.setCursor(412, 38);
    tft.println("DEFAULT");
}

void sampleWindow()
{
    screen = 4;
    tft.fillRect(0,32,DISPLAY_WIDTH-1,DISPLAY_HEIGHT-33,BACK_GROUND_COLOR);

    tft.drawRect(0,32,DISPLAY_WIDTH-80,DISPLAY_HEIGHT-33,FRAME_COLOR);
    
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<3;j++)
        {
            tft.fillRoundRect((j*52)+231,(i*52)+51,45,45,6,BUTTON_COLOR_1);
        }
    }
    for(int i=0;i<3;i++)
    {   
        tft.setTextColor(0x0000);
        tft.setFontScale(TEXT_SIZE_MEDIUM);
        tft.setCursor((i*52)+246,57);
        tft.println(keyboard[i]);
        tft.setCursor((i*52)+246,57+51);
        tft.println(keyboard[i+3]);
        tft.setCursor((i*52)+246,58+51*2);
        tft.println(keyboard[i+6]);
    }
    tft.setCursor(246,58+51*3);
    tft.println(0);
    tft.setCursor(52+246,58+51*3);
    tft.println('C');
    tft.setCursor(2*52+236,58+51*3);
    tft.println("<<");
    tft.drawRect(220,40,172,223,FRAME_COLOR);
    tft.drawRect(219,39,174,225,FRAME_COLOR);
    tft.drawRect(218,38,176,227,FRAME_COLOR);

    tft.setTextColor(0xffff);
    tft.setFontScale(TEXT_SIZE_SMALL);
    tft.setCursor(41,50);
    tft.println("Number of Samples");

    tft.setTextColor(0x0000,0xffff);
    tft.setFontScale(1.5);

    key[4] = samples;

    tft.setCursor(69,82);
    sprintf(u_sample,"%5.0d",samples);    
    tft.print(u_sample);
    
    tft.drawRect(67,80,83,35,FRAME_COLOR);
    tft.drawRect(66,79,85,37,FRAME_COLOR);
    tft.drawRect(65,78,87,39,FRAME_COLOR);

    tft.fillRoundRect(DISPLAY_WIDTH-77,112,74,28 ,5,BUTTON_COLOR_1);
    tft.fillRoundRect(DISPLAY_WIDTH-77,165,74,28,5,BUTTON_COLOR_1);  
    tft.setTextColor(BUTTON_TEXT_COLOR);
    tft.setFontScale(TEXT_SIZE_SMALL);
    tft.setCursor(432,118);
    tft.println("OK");
    tft.setCursor(417,171);
    tft.println("Cancel");
}

void editWindow(uint8_t stage)
{
    screen = 3;
    tft.fillRect(0,32,DISPLAY_WIDTH-1,DISPLAY_HEIGHT-33,BACK_GROUND_COLOR);
    tft.drawRect(DISPLAY_WIDTH-78,32,77,23,FRAME_COLOR);
    tft.setCursor(DISPLAY_WIDTH-68,35);
    tft.println("Stage");
    tft.setCursor(DISPLAY_WIDTH-23,35);
    tft.println(stage);

    tft.drawRect(0,32,DISPLAY_WIDTH-80,DISPLAY_HEIGHT-33,FRAME_COLOR);
    
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<3;j++)
        {
            tft.fillRoundRect((j*52)+231,(i*52)+51,45,45,6,0x6767);
        }
    }
    for(int i=0;i<3;i++)
    {   
        tft.setTextColor(0x0000);
        tft.setFontScale(TEXT_SIZE_MEDIUM);
        tft.setCursor((i*52)+246,57);
        tft.println(keyboard[i]);
        tft.setCursor((i*52)+246,57+51);
        tft.println(keyboard[i+3]);
        tft.setCursor((i*52)+246,58+51*2);
        tft.println(keyboard[i+6]);
    }
    tft.setCursor(246,58+51*3);
    tft.println(0);
    tft.setCursor(52+246,58+51*3);
    tft.println('C');
    tft.setCursor(2*52+236,58+51*3);
    tft.println("<<");
    tft.drawRect(220,40,172,223,FRAME_COLOR);
    tft.drawRect(219,39,174,225,FRAME_COLOR);
    tft.drawRect(218,38,176,227,FRAME_COLOR);

    tft.setTextColor(0xffff);
    tft.setFontScale(TEXT_SIZE_SMALL);
    tft.setCursor(5,50);
    tft.println("Volume  (ul)");
    tft.setCursor(5,173);
    tft.println("Vacuum");
    tft.setCursor(11,191);
    tft.println("Time");
    tft.setCursor(15,209);
    tft.println("(s)");
    tft.setCursor(75,144);
    tft.println("Low");
    tft.setCursor(75,191);
    tft.println("Off");
    tft.setCursor(75,238);
    tft.println("High");

    tft.drawLine(63,152,63,247,FRAME_COLOR);
    tft.drawLine(63,152,68,152,FRAME_COLOR);
    tft.drawLine(63,199,68,199,FRAME_COLOR);
    tft.drawLine(63,247,68,247,FRAME_COLOR);

    tft.setTextColor(0x0000,0xffff);
    tft.setFontScale(1.5);

    key[0] = volume[stage-1];
    key[1] = vacuum_low[stage-1];
    key[2] = vacuum_off[stage-1];
    key[3] = vacuum_high[stage-1];

    tft.setCursor(116,42);
    sprintf(u1,"%5.0d",volume[stage-1]);    
    tft.print(u1);
    tft.setCursor(116,136);
    sprintf(u2,"%5.0d",vacuum_low[stage-1]);    
    tft.print(u2);
    tft.setCursor(116,183);
    sprintf(u3,"%5.0d",vacuum_off[stage-1]);    
    tft.print(u3);
    tft.setCursor(116,230);
    sprintf(u4,"%5.0d",vacuum_high[stage-1]);    
    tft.print(u4);

    tft.fillRoundRect(DISPLAY_WIDTH-77,112,74,28 ,5,BUTTON_COLOR_1);
    tft.fillRoundRect(DISPLAY_WIDTH-77,165,74,28,5,BUTTON_COLOR_1);  
    tft.setTextColor(BUTTON_TEXT_COLOR);
    tft.setFontScale(TEXT_SIZE_SMALL);
    tft.setCursor(432,118);
    tft.println("OK");
    tft.setCursor(417,171);
    tft.println("Cancel");

    n=0;
    tft.drawRect(114,40,83,35,FRAME_COLOR);
    tft.drawRect(113,39,85,37,FRAME_COLOR);
    tft.drawRect(112,38,87,39,FRAME_COLOR);
}

void runScreen()
{
    screen = 5;
    
    tft.fillScreen(BACK_GROUND_COLOR);
    tft.drawRect(0,0,DISPLAY_WIDTH-1,30,FRAME_COLOR);
    tft.fillRect(3,3,DISPLAY_WIDTH-7,30-6,FRAME_COLOR);
    tft.drawRect(0,DISPLAY_HEIGHT-30,DISPLAY_WIDTH-1,29,FRAME_COLOR);

    tft.setTextColor(TEXT_COLOR_1);
    tft.setFontScale(TEXT_SIZE_SMALL);
    tft.setCursor(225,7);
    tft.println("RUN");
    
    tft.fillRoundRect(199,DISPLAY_HEIGHT-30+2,75,25,5,BUTTON_COLOR_1);
    tft.setTextColor(BUTTON_TEXT_COLOR);
    tft.setFontScale(TEXT_SIZE_SMALL);

    tft.setCursor(222, DISPLAY_HEIGHT-30+7);
    tft.println("STOP");

    for(int i=0; i<4;i++)
    {   
        tft.drawRect(i*((DISPLAY_WIDTH/4)-10),33,(DISPLAY_WIDTH/4)-10,DISPLAY_HEIGHT-66,FRAME_COLOR);
        tft.fillRect(i*((DISPLAY_WIDTH/4)-10)+2,35,(DISPLAY_WIDTH/4)-14,27,FRAME_COLOR);

        tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55,177,40,FRAME_COLOR);
        tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55,177,41,FRAME_COLOR);
        tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55,177,42,FRAME_COLOR);
        
        tft.setFontScale(TEXT_SIZE_SMALL);
        //tft.setTextColor(0xffff);
        tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+27, 169);
        tft.println("WAITING");

        tft.setFontScale(TEXT_SIZE_MEDIUM);
        tft.setTextColor(0xe0e0);
        if(m[i]<10)
        {  
            tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+18, 76);
            tft.println(0);
            tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+33, 76);
            tft.println(m[i]);
        }
        else
        {
            tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+18, 76);
            tft.println(m[i]);
        }

        tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+48, 76);
        tft.println(":");

        if(s[i]<10)
        {
            tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+63, 76);
            tft.println(0);
            tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+78, 76);
            tft.println(s[i]);
        }
        else
        {
            tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+63, 76);
            tft.println(s[i]);
        }
    }

    tft.fillRect((4*((DISPLAY_WIDTH/4)-10))+2,33,37,DISPLAY_HEIGHT-66,FRAME_COLOR);
    tft.fillTriangle(451,120,451,150,471,135,0xffff);

    if(newprogram1==1)
    {
        tft.setTextColor(TEXT_COLOR_1);
        tft.setFontScale(TEXT_SIZE_SMALL);
        tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+20, 40);
        tft.println("Priming 1");
        tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+20, 40);
        tft.println("Priming 2");
        tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+24, 40);
        tft.println("Load OBS");
        tft.setCursor(3*((DISPLAY_WIDTH/4)-10)+32, 40);
        tft.println("Wash 1");
    }

    if(newprogram2==1)
    {
        tft.setTextColor(TEXT_COLOR_1);
        tft.setFontScale(TEXT_SIZE_SMALL);
        tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+15, 40);
        tft.println("Activation");
        tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+20, 40);
        tft.println("Washing 1");
        tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+30, 40);
        tft.println("Sample");
        tft.setCursor(3*((DISPLAY_WIDTH/4)-10)+20, 40);
        tft.println("Washing 2");
    }

    //runScreenUpdate(run_scr);
}

void runScreenUpdate(int run_scr)
{
    if(group_stage==1)
    {
        if(run_scr==1)
        {
            tft.fillRect(0*((DISPLAY_WIDTH/4)-10)+2,35+29,(DISPLAY_WIDTH/4)-14,DISPLAY_HEIGHT-66-33,BACK_GROUND_COLOR);
            tft.drawCircle(0*((DISPLAY_WIDTH/4)-10)+55,177,40,0x1919);
            tft.drawCircle(0*((DISPLAY_WIDTH/4)-10)+55,177,41,0x1919);
            tft.drawCircle(0*((DISPLAY_WIDTH/4)-10)+55,177,42,0x1919);
            tft.setFontScale(TEXT_SIZE_SMALL);
            tft.setTextColor(0x1919);
            tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+27, 169);
            tft.println("PUMPING");
           
            tft.setFontScale(TEXT_SIZE_MEDIUM);
            tft.setTextColor(0xe0e0);
            if(m[0]<10)
            {  
                tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+18, 76);
                tft.println(0);
                tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+33, 76);
                tft.println(m[0]);
            }
            else
            {
                tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+18, 76);
                tft.println(m[0]);
            }

            tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+48, 76);
            tft.println(":");

            if(s[0]<10)
            {
                tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+63, 76);
                tft.println(0);
                tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+78, 76);
                tft.println(s[0]);
            }
            else
            {
                tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+63, 76);
                tft.println(s[0]);
            }
        }

        if(run_scr==2)
        {
            tft.fillRect(0*((DISPLAY_WIDTH/4)-10)+2,35+29,(DISPLAY_WIDTH/4)-14,DISPLAY_HEIGHT-66-33,0xd6d6);
            tft.drawCircle(0*((DISPLAY_WIDTH/4)-10)+55,177,40,0x4545);
            tft.drawCircle(0*((DISPLAY_WIDTH/4)-10)+55,177,41,0x4545);
            tft.drawCircle(0*((DISPLAY_WIDTH/4)-10)+55,177,42,0x4545);
            tft.setFontScale(TEXT_SIZE_SMALL);
            tft.setTextColor(0x4545);
            tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+20, 169);
            tft.println("COMPLETED");

            tft.fillRect(1*((DISPLAY_WIDTH/4)-10)+2,35+29,(DISPLAY_WIDTH/4)-14,DISPLAY_HEIGHT-66-33,BACK_GROUND_COLOR);
            tft.drawCircle(1*((DISPLAY_WIDTH/4)-10)+55,177,40,0x1919);
            tft.drawCircle(1*((DISPLAY_WIDTH/4)-10)+55,177,41,0x1919);
            tft.drawCircle(1*((DISPLAY_WIDTH/4)-10)+55,177,42,0x1919);
            tft.setFontScale(TEXT_SIZE_SMALL);
            tft.setTextColor(0x1919);
            tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+27, 169);
            tft.println("PUMPING");   

            tft.setFontScale(TEXT_SIZE_MEDIUM);
            tft.setTextColor(0xe0e0);
            if(m[1]<10)
            {  
                tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+18, 76);
                tft.println(0);
                tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+33, 76);
                tft.println(m[1]);
            }
            else
            {
                tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+18, 76);
                tft.println(m[1]);
            }

            tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+48, 76);
            tft.println(":");

            if(s[1]<10)
            {
                tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+63, 76);
                tft.println(0);
                tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+78, 76);
                tft.println(s[1]);
            }
            else
            {
                tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+63, 76);
                tft.println(s[1]);
            }
        }

        if(run_scr==3)
        {
            for(int i=0;i<2;i++)
            {
                tft.fillRect(i*((DISPLAY_WIDTH/4)-10)+2,35+29,(DISPLAY_WIDTH/4)-14,DISPLAY_HEIGHT-66-33,0xd6d6);
                tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55,177,40,0x4545);
                tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55,177,41,0x4545);
                tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55,177,42,0x4545);
                tft.setFontScale(TEXT_SIZE_SMALL);
                tft.setTextColor(0x4545);
                tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+20, 169);
                tft.println("COMPLETED");
            }
            
            tft.fillRect(2*((DISPLAY_WIDTH/4)-10)+2,35+29,(DISPLAY_WIDTH/4)-14,DISPLAY_HEIGHT-66-33,BACK_GROUND_COLOR);   
            tft.drawCircle(2*((DISPLAY_WIDTH/4)-10)+55,177,40,0x1919);
            tft.drawCircle(2*((DISPLAY_WIDTH/4)-10)+55,177,41,0x1919);
            tft.drawCircle(2*((DISPLAY_WIDTH/4)-10)+55,177,42,0x1919);
            tft.setFontScale(TEXT_SIZE_SMALL);
            tft.setTextColor(0x1919);
            tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+27, 169);
            tft.println("PUMPING");

            tft.setFontScale(TEXT_SIZE_MEDIUM);
            tft.setTextColor(0xe0e0);
            if(m[2]<10)
            {  
                tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+18, 76);
                tft.println(0);
                tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+33, 76);
                tft.println(m[2]);
            }
            else
            {
                tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+18, 76);
                tft.println(m[2]);
            }

            tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+48, 76);
            tft.println(":");

            if(s[2]<10)
            {
                tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+63, 76);
                tft.println(0);
                tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+78, 76);
                tft.println(s[2]);
            }
            else
            {
                tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+63, 76);
                tft.println(s[2]);
            }
        }

        if(run_scr==4)
        {
            for(int i=0;i<3;i++)
            {
                tft.fillRect(i*((DISPLAY_WIDTH/4)-10)+2,35+29,(DISPLAY_WIDTH/4)-14,DISPLAY_HEIGHT-66-33,0xd6d6);
                tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55,177,40,0x4545);
                tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55,177,41,0x4545);
                tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55,177,42,0x4545);
                tft.setFontScale(TEXT_SIZE_SMALL);
                tft.setTextColor(0x4545);
                tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+20, 169);
                tft.println("COMPLETED");
            }

            tft.fillRect(3*((DISPLAY_WIDTH/4)-10)+2,35+29,(DISPLAY_WIDTH/4)-14,DISPLAY_HEIGHT-66-33,BACK_GROUND_COLOR);  
            tft.drawCircle(3*((DISPLAY_WIDTH/4)-10)+55,177,40,0x1919);
            tft.drawCircle(3*((DISPLAY_WIDTH/4)-10)+55,177,41,0x1919);
            tft.drawCircle(3*((DISPLAY_WIDTH/4)-10)+55,177,42,0x1919);
            tft.setFontScale(TEXT_SIZE_SMALL);
            tft.setTextColor(0x1919);
            tft.setCursor(3*((DISPLAY_WIDTH/4)-10)+27, 169);
            tft.println("PUMPING"); 

            tft.setFontScale(TEXT_SIZE_MEDIUM);
            tft.setTextColor(0xe0e0);
            if(m[3]<10)
            {  
                tft.setCursor(3*((DISPLAY_WIDTH/4)-10)+18, 76);
                tft.println(0);
                tft.setCursor(3*((DISPLAY_WIDTH/4)-10)+33, 76);
                tft.println(m[3]);
            }
            else
            {
                tft.setCursor(3*((DISPLAY_WIDTH/4)-10)+18, 76);
                tft.println(m[3]);
            }

            tft.setCursor(3*((DISPLAY_WIDTH/4)-10)+48, 76);
            tft.println(":");

            if(s[3]<10)
            {
                tft.setCursor(3*((DISPLAY_WIDTH/4)-10)+63, 76);
                tft.println(0);
                tft.setCursor(3*((DISPLAY_WIDTH/4)-10)+78, 76);
                tft.println(s[3]);
            }
            else
            {
                tft.setCursor(3*((DISPLAY_WIDTH/4)-10)+63, 76);
                tft.println(s[3]);
            }
        }

        if(run_scr>4)
        {
            for(uint8_t i=0; i<4; i++)
            {
                tft.fillRect(i*((DISPLAY_WIDTH/4)-10)+2,35+29,(DISPLAY_WIDTH/4)-14,DISPLAY_HEIGHT-66-33,0xd6d6);
                tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55,177,40,0x4545);
                tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55,177,41,0x4545);
                tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55,177,42,0x4545);
                tft.setFontScale(TEXT_SIZE_SMALL);
                tft.setTextColor(0x4545);
                tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+20, 169);
                tft.println("COMPLETED");      
            }
            if(run_switch_screen==1)
            {
                run_switch_screen = 0;
                group_stage = 2;
                tft.fillRect(0,33,DISPLAY_WIDTH,DISPLAY_HEIGHT-66,BACK_GROUND_COLOR);
                uint8_t run_count;
                if(newprogram1==1)
                    run_count=3;
                if(newprogram2==1)
                    run_count=1;
                for(int i=0; i<run_count;i++)
                {   
                    tft.drawRect(i*((DISPLAY_WIDTH/4)-10)+39,33,(DISPLAY_WIDTH/4)-10,DISPLAY_HEIGHT-66,FRAME_COLOR);
                    tft.fillRect(i*((DISPLAY_WIDTH/4)-10)+2+39,35,(DISPLAY_WIDTH/4)-14,27,FRAME_COLOR);

                    if(i<3)
                    {
                        tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55+39,177,40,FRAME_COLOR);
                        tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55+39,177,41,FRAME_COLOR);
                        tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55+39,177,42,FRAME_COLOR);
                        
                        tft.setFontScale(TEXT_SIZE_SMALL);
                        //tft.setTextColor(0xffff);
                        tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+27+39, 169);
                        tft.println("WAITING");

                        tft.setFontScale(TEXT_SIZE_MEDIUM);
                        tft.setTextColor(0xe0e0);
                        if(m[i+4]<10)
                        {  
                            tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+18+39, 76);
                            tft.println(0);
                            tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+33+39, 76);
                            tft.println(m[i+4]);
                        }
                        else
                        {
                            tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+18+39, 76);
                            tft.println(m[i+4]);
                        }

                        tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+48+39, 76);
                        tft.println(":");

                        if(s[i+4]<10)
                        {
                            tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+63+39, 76);
                            tft.println(0);
                            tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+78+39, 76);
                            tft.println(s[i+4]);
                        }
                        else
                        {
                            tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+63+39, 76);
                            tft.println(s[i+4]);
                        }
                    }
                }
                tft.fillRect(0,33,37,DISPLAY_HEIGHT-66,FRAME_COLOR);
                tft.fillTriangle(28,120,28,150,8,135,0xffff);
                
                tft.setTextColor(TEXT_COLOR_1);
                tft.setFontScale(TEXT_SIZE_SMALL);
                if(newprogram1==1)
                {
                    tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+11+39, 40);
                    tft.println("DMT Removal");
                    tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+31+39, 40);
                    tft.println("Wash 2");
                    tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+28+39, 40);
                    tft.println("Elution");
                }
                if(newprogram2==1)
                {
                    tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+26+39, 40);
                    tft.println("Elution");;
                }
            }
        }

    }

    if(group_stage==2)
    {
        if(run_scr==5)
        {
            tft.fillRect(0*((DISPLAY_WIDTH/4)-10)+2+39,35+29,(DISPLAY_WIDTH/4)-14,DISPLAY_HEIGHT-66-33,BACK_GROUND_COLOR);
            tft.drawCircle(0*((DISPLAY_WIDTH/4)-10)+55+39,177,40,0x1919);
            tft.drawCircle(0*((DISPLAY_WIDTH/4)-10)+55+39,177,41,0x1919);
            tft.drawCircle(0*((DISPLAY_WIDTH/4)-10)+55+39,177,42,0x1919);
            tft.setFontScale(TEXT_SIZE_SMALL);
            tft.setTextColor(0x1919);
            tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+27+39, 169);
            tft.println("PUMPING");

            tft.setFontScale(TEXT_SIZE_MEDIUM);
            tft.setTextColor(0xe0e0);
            if(m[4]<10)
            {  
                tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+18+39, 76);
                tft.println(0);
                tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+33+39, 76);
                tft.println(m[4]);
            }
            else
            {
                tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+18+39, 76);
                tft.println(m[4]);
            }

            tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+48+39, 76);
            tft.println(":");

            if(s[4]<10)
            {
                tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+63+39, 76);
                tft.println(0);
                tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+78+39, 76);
                tft.println(s[4]);
            }
            else
            {
                tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+63+39, 76);
                tft.println(s[4]);
            }
        }

        if(run_scr==6)
        {
            tft.fillRect(0*((DISPLAY_WIDTH/4)-10)+2+39,35+29,(DISPLAY_WIDTH/4)-14,DISPLAY_HEIGHT-66-33,0xd6d6);
            tft.drawCircle(0*((DISPLAY_WIDTH/4)-10)+55+39,177,40,0x4545);
            tft.drawCircle(0*((DISPLAY_WIDTH/4)-10)+55+39,177,41,0x4545);
            tft.drawCircle(0*((DISPLAY_WIDTH/4)-10)+55+39,177,42,0x4545);
            tft.setFontScale(TEXT_SIZE_SMALL);
            tft.setTextColor(0x4545);
            tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+20+39, 169);
            tft.println("COMPLETED");

            tft.fillRect(1*((DISPLAY_WIDTH/4)-10)+2+39,35+29,(DISPLAY_WIDTH/4)-14,DISPLAY_HEIGHT-66-33,BACK_GROUND_COLOR);
            tft.drawCircle(1*((DISPLAY_WIDTH/4)-10)+55+39,177,40,0x1919);
            tft.drawCircle(1*((DISPLAY_WIDTH/4)-10)+55+39,177,41,0x1919);
            tft.drawCircle(1*((DISPLAY_WIDTH/4)-10)+55+39,177,42,0x1919);
            tft.setFontScale(TEXT_SIZE_SMALL);
            tft.setTextColor(0x1919);
            tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+27+39, 169);
            tft.println("PUMPING");

            tft.setFontScale(TEXT_SIZE_MEDIUM);
            tft.setTextColor(0xe0e0);
            if(m[5]<10)
            {  
                tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+18+39, 76);
                tft.println(0);
                tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+33+39, 76);
                tft.println(m[5]);
            }
            else
            {
                tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+18+39, 76);
                tft.println(m[5]);
            }

            tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+48+39, 76);
            tft.println(":");

            if(s[5]<10)
            {
                tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+63+39, 76);
                tft.println(0);
                tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+78+39, 76);
                tft.println(s[5]);
            }
            else
            {
                tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+63+39, 76);
                tft.println(s[5]);
            }
        }

        if(run_scr==7)
        {
            for(int i=0;i<2;i++)
            {
                tft.fillRect(i*((DISPLAY_WIDTH/4)-10)+2+39,35+29,(DISPLAY_WIDTH/4)-14,DISPLAY_HEIGHT-66-33,0xd6d6);
                tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55+39,177,40,0x4545);
                tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55+39,177,41,0x4545);
                tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55+39,177,42,0x4545);
                tft.setFontScale(TEXT_SIZE_SMALL);
                tft.setTextColor(0x4545);
                tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+20+39, 169);
                tft.println("COMPLETED");
            }

            tft.fillRect(2*((DISPLAY_WIDTH/4)-10)+2+39,35+29,(DISPLAY_WIDTH/4)-14,DISPLAY_HEIGHT-66-33,BACK_GROUND_COLOR);
            tft.drawCircle(2*((DISPLAY_WIDTH/4)-10)+55+39,177,40,0x1919);
            tft.drawCircle(2*((DISPLAY_WIDTH/4)-10)+55+39,177,41,0x1919);
            tft.drawCircle(2*((DISPLAY_WIDTH/4)-10)+55+39,177,42,0x1919);
            tft.setFontScale(TEXT_SIZE_SMALL);
            tft.setTextColor(0x1919);
            tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+27+39, 169);
            tft.println("PUMPING");

            tft.setFontScale(TEXT_SIZE_MEDIUM);
            tft.setTextColor(0xe0e0);
            if(m[6]<10)
            {  
                tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+18+39, 76);
                tft.println(0);
                tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+33+39, 76);
                tft.println(m[6]);
            }
            else
            {
                tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+18+39, 76);
                tft.println(m[6]);
            }

            tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+48+39, 76);
            tft.println(":");

            if(s[6]<10)
            {
                tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+63+39, 76);
                tft.println(0);
                tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+78+39, 76);
                tft.println(s[6]);
            }
            else
            {
                tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+63+39, 76);
                tft.println(s[6]);
            }
        }
    }
}

//////////////////// DISPLAY - END ////////////////////

///////////////////  PRESS - START  ///////////////////
void pressButton()
{
    if(enablepress==1)
    {
        if(tft.touchDetect())
        {  
            delay(500);
            tft.touchReadPixel(&tx, &ty);
            if(screen==0) //main
            {
                if(tx>5 && tx<109 && ty>119 && ty<149) //new program pressed 
                {
                    tx=0;
                    ty=0;

                    programScreen();
                }
                if(tx>126 && tx<232 && ty>119 && ty<149) //history pressed 
                {
                    tx=0;
                    ty=0;
                    //switchScreen(...);
                }
                if(tx>247 && tx<353 && ty>119 && ty<149) //manual pressed 
                {
                    tx=0;
                    ty=0;
                    //switchScreen(...);
                }
                if(tx>368 && tx<474 && ty>119 && ty<149) //setting pressed 
                {
                    tx=0;
                    ty=0;
                    //switchScreen(...);
                }
            }

            if(screen==1) //new program
            {
                if(tx>=162 && tx<=316 && ty>=38 && ty<=234) //default program 2 pressed 
                {
                    tx=0;
                    ty=0;

                    tft.drawRect(1,38,154,196,BACK_GROUND_COLOR);
                    tft.drawRect(2,39,152,194,BACK_GROUND_COLOR);
                    tft.drawRect(3,40,150,192,BACK_GROUND_COLOR);
                    tft.drawRect(4,41,148,190,BACK_GROUND_COLOR);

                    tft.drawRect(162,38,154,196,0xa7a7);
                    tft.drawRect(163,39,152,194,0xa7a7);
                    tft.drawRect(164,40,150,192,0xa7a7);
                    tft.drawRect(165,41,148,190,0xa7a7);

                    tft.drawRect(323,38,154,196,BACK_GROUND_COLOR);
                    tft.drawRect(324,39,152,194,BACK_GROUND_COLOR);
                    tft.drawRect(325,40,150,192,BACK_GROUND_COLOR);
                    tft.drawRect(326,41,148,190,BACK_GROUND_COLOR);

                    tft.drawCircle((1*(DISPLAY_WIDTH/3)+79),135,50,0xa7a7);
                    tft.drawCircle((0*(DISPLAY_WIDTH/3)+79),135,50,FRAME_COLOR);
                    tft.drawCircle((2*(DISPLAY_WIDTH/3)+79),135,50,FRAME_COLOR);
                    tft.drawCircle((1*(DISPLAY_WIDTH/3)+79),135,51,0xa7a7);
                    tft.drawCircle((0*(DISPLAY_WIDTH/3)+79),135,51,FRAME_COLOR);
                    tft.drawCircle((2*(DISPLAY_WIDTH/3)+79),135,51,FRAME_COLOR);
                    tft.drawCircle((1*(DISPLAY_WIDTH/3)+79),135,52,0xa7a7);
                    tft.drawCircle((0*(DISPLAY_WIDTH/3)+79),135,52,FRAME_COLOR);
                    tft.drawCircle((2*(DISPLAY_WIDTH/3)+79),135,52,FRAME_COLOR);

                    newprogram1 = 0;
                    newprogram2 = 1;
                    newprogram3 = 0;
                }

                if(tx>=1 && tx<=155 && ty>=38 && ty<=234) //default program 1 pressed 
                {
                    tx=0;
                    ty=0;

                    tft.drawRect(1,38,154,196,0xa7a7);
                    tft.drawRect(2,39,152,194,0xa7a7);
                    tft.drawRect(3,40,150,192,0xa7a7);
                    tft.drawRect(4,41,148,190,0xa7a7);

                    tft.drawRect(162,38,154,196,BACK_GROUND_COLOR);
                    tft.drawRect(163,39,152,194,BACK_GROUND_COLOR);
                    tft.drawRect(164,40,150,192,BACK_GROUND_COLOR);
                    tft.drawRect(165,41,148,190,BACK_GROUND_COLOR);

                    tft.drawRect(323,38,154,196,BACK_GROUND_COLOR);
                    tft.drawRect(324,39,152,194,BACK_GROUND_COLOR);
                    tft.drawRect(325,40,150,192,BACK_GROUND_COLOR);
                    tft.drawRect(326,41,148,190,BACK_GROUND_COLOR);

                    tft.drawCircle((0*(DISPLAY_WIDTH/3)+79),135,50,0xa7a7);
                    tft.drawCircle((1*(DISPLAY_WIDTH/3)+79),135,50,FRAME_COLOR);
                    tft.drawCircle((2*(DISPLAY_WIDTH/3)+79),135,50,FRAME_COLOR);
                    tft.drawCircle((0*(DISPLAY_WIDTH/3)+79),135,51,0xa7a7);
                    tft.drawCircle((1*(DISPLAY_WIDTH/3)+79),135,51,FRAME_COLOR);
                    tft.drawCircle((2*(DISPLAY_WIDTH/3)+79),135,51,FRAME_COLOR);
                    tft.drawCircle((0*(DISPLAY_WIDTH/3)+79),135,52,0xa7a7);
                    tft.drawCircle((1*(DISPLAY_WIDTH/3)+79),135,52,FRAME_COLOR);
                    tft.drawCircle((2*(DISPLAY_WIDTH/3)+79),135,52,FRAME_COLOR);

                    newprogram1 = 1;
                    newprogram2 = 0;
                    newprogram3 = 0;
                }

                if(tx>323 && tx<=477 && ty>=38 && ty<=234) //custom program pressed 
                {
                    tx=0;
                    ty=0;

                    tft.drawRect(1,38,154,196,BACK_GROUND_COLOR);
                    tft.drawRect(2,39,152,194,BACK_GROUND_COLOR);
                    tft.drawRect(3,40,150,192,BACK_GROUND_COLOR);
                    tft.drawRect(4,41,148,190,BACK_GROUND_COLOR);

                    tft.drawRect(162,38,154,196,BACK_GROUND_COLOR);
                    tft.drawRect(163,39,152,194,BACK_GROUND_COLOR);
                    tft.drawRect(164,40,150,192,BACK_GROUND_COLOR);
                    tft.drawRect(165,41,148,190,BACK_GROUND_COLOR);

                    tft.drawRect(323,38,154,196,0xa7a7);
                    tft.drawRect(324,39,152,194,0xa7a7);
                    tft.drawRect(325,40,150,192,0xa7a7);
                    tft.drawRect(326,41,148,190,0xa7a7);

                    tft.drawCircle((2*(DISPLAY_WIDTH/3)+79),135,50,0xa7a7);
                    tft.drawCircle((0*(DISPLAY_WIDTH/3)+79),135,50,FRAME_COLOR);
                    tft.drawCircle((1*(DISPLAY_WIDTH/3)+79),135,50,FRAME_COLOR);
                    tft.drawCircle((2*(DISPLAY_WIDTH/3)+79),135,51,0xa7a7);
                    tft.drawCircle((0*(DISPLAY_WIDTH/3)+79),135,51,FRAME_COLOR);
                    tft.drawCircle((1*(DISPLAY_WIDTH/3)+79),135,51,FRAME_COLOR);
                    tft.drawCircle((2*(DISPLAY_WIDTH/3)+79),135,52,0xa7a7);
                    tft.drawCircle((0*(DISPLAY_WIDTH/3)+79),135,52,FRAME_COLOR);
                    tft.drawCircle((1*(DISPLAY_WIDTH/3)+79),135,52,FRAME_COLOR);
                    
                    newprogram1 = 0;
                    newprogram2 = 0;
                    newprogram3 = 1;
                }

                if(tx>2 && tx<77 && ty>244 && ty<269) //back pressed   
                {
                    tx=0;
                    ty=0;

                    mainScreen();
                }
                
                if(tx>402 && tx<477 && ty>244 && ty<269) //next pressed 
                {
                    tx=0;
                    ty=0;

                    if(newprogram1 == 1) //default 1 program 
                    {
                        // tft.fillRoundRect(90,35,300,200,5,0xd6d6);
                        // tft.fillRoundRect(90,35,300,35,5,0x2525);
                        // tft.setTextColor(0xffff);
                        // tft.setFontScale(TEXT_SIZE_SMALL);
                        // tft.setCursor(250, 44);
                        // tft.println("...");
                        // tft.setTextColor(0x0000);
                        // tft.setCursor(170, 100);
                        // tft.println("Please select EZ type !");
                        // tft.fillRoundRect(50,189,75,25,5,0x2525);
                        // tft.setTextColor(0xffff);
                        // tft.setCursor(80, 193);
                        // tft.println("EZ50");
                        // tft.fillRoundRect(150,189,75,25,5,0x2525);
                        // tft.setTextColor(0xffff);
                        // tft.setCursor(180, 193);
                        // tft.println("EZ100");

                        // while(1)
                        // {   
                        //     if(tft.touchDetect())
                        //     {  
                        //         tft.touchReadPixel(&tx, &ty);
                        //         if(tx>135 && tx<210 && ty>189 && ty<214)
                        //         {
                        //             tx=0;
                        //             ty=0;
                        //         }
                        //     }
                        // }



                        for(int ep=0; ep<7; ep++)
                        {   
                            if(EEPROM.read(ep+300)==1)
                                volume[ep] = EEPROM.read(ep+500)*10 + EEPROM.read(ep+600);
                            else
                                volume[ep] = EEPROM.read(ep+500);

                            if(EEPROM.read(ep+300+15)==1)
                                vacuum_off[ep] = EEPROM.read(ep+500+15)*10 + EEPROM.read(ep+600+15);
                            else
                                vacuum_off[ep] = EEPROM.read(ep+500+15);
                            
                            if(EEPROM.read(ep+300+30)==1)
                                vacuum_low[ep] = EEPROM.read(ep+500+30)*10 + EEPROM.read(ep+600+30);
                            else
                                vacuum_low[ep] = EEPROM.read(ep+500+30);
                            
                            if(EEPROM.read(ep+300+30)==1)
                                vacuum_high[ep] = EEPROM.read(ep+500+45)*10 + EEPROM.read(ep+600+45);
                            else
                                vacuum_high[ep] = EEPROM.read(ep+500+45);
                        }
                        configScreen();
                        delayPress = 1;
                        stages = 7;
                    }

                    if(newprogram2 == 1) //default 2 program 
                    {
                        for(int ep=0; ep<5; ep++)
                        {
                            if(EEPROM.read(ep+400)==1)
                                volume[ep] = EEPROM.read(ep+700)*10 + EEPROM.read(ep+800);
                            else
                                volume[ep] = EEPROM.read(ep+700);

                            if(EEPROM.read(ep+400+15)==1)
                                vacuum_off[ep] = EEPROM.read(ep+700+15)*10 + EEPROM.read(ep+800+15);
                            else
                                vacuum_off[ep] = EEPROM.read(ep+700+15);
                            
                            if(EEPROM.read(ep+400+30)==1)
                                vacuum_low[ep] = EEPROM.read(ep+700+30)*10 + EEPROM.read(ep+800+30);
                            else
                                vacuum_low[ep] = EEPROM.read(ep+700+30);
                            
                            if(EEPROM.read(ep+400+30)==1)
                                vacuum_high[ep] = EEPROM.read(ep+700+45)*10 + EEPROM.read(ep+800+45);
                            else
                                vacuum_high[ep] = EEPROM.read(ep+700+45);
                        }

                        configScreen();
                        delayPress = 1;
                        stages = 5;
                    }

                    if(newprogram3 == 1) //custom program 
                    {
                        
                    }
                }    
            }

            if(screen==2) //Configuration 
            {     
                if(group_stage == 1)
                {   
                    if(tx>140 && tx<190 && ty>200 && ty<225) //edit 2 pressed
                    {
                        tx=0;
                        ty=0;
                        if(stages>1)
                        {
                            stage_config = 1;
                            editWindow(2);
                        }
                    }

                    else if(tx>30 && tx<80 && ty>200 && ty<225) //edit 1 pressed 
                    {
                        delay(300);
                        //tx=0;
                        //ty=0;

                        stage_config = 0;
                        editWindow(1);
                    }

                    else if(tx>250 && tx<300 && ty>200 && ty<225) //edit 3 pressed
                    {
                        tx=0;
                        ty=0;
                        if(stages>2)
                        {
                            stage_config = 2;
                            editWindow(3);
                        }
                    }
                    
                    else if(tx>360 && tx<410 && ty>200 && ty<225) //edit 4 pressed
                    {
                        tx=0;
                        ty=0;
                        if(stages>3)
                        {
                            stage_config = 3;
                            editWindow(4);
                        }
                    }
                    
                    else if(tx>442 && tx<479 && ty>60 && ty<234) //arrow pressed
                    {
                        tx=0;
                        ty=0;
                        if(stages>4) 
                        {
                            group_stage = 2;
                            tft.fillRect(0,60,DISPLAY_WIDTH,DISPLAY_HEIGHT-98,BACK_GROUND_COLOR);
                            for(int i=0; i<4;i++)
                            {   
                                tft.drawRect(i*((DISPLAY_WIDTH/4)-10)+39,60,(DISPLAY_WIDTH/4)-10,DISPLAY_HEIGHT-98,FRAME_COLOR);
                                tft.drawRect(i*((DISPLAY_WIDTH/4)-10)+39,60,(DISPLAY_WIDTH/4)-10,27,FRAME_COLOR);
                            }
                            tft.fillRect(0,60,37,DISPLAY_HEIGHT-98,FRAME_COLOR);
                            tft.fillTriangle(28,135,28,165,8,150,0xffff);
                            for(int i=0; i<stages-4; i++)
                            {    
                                if(i==4)    break;
                                tft.setTextColor(TEXT_COLOR_1);
                                tft.setFontScale(TEXT_SIZE_SMALL);
                                // tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+34+39, 64);
                                // tft.println("STEP");
                                // tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+70+39, 64);
                                // tft.println(i+5);

                                tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+5+39, 105);
                                tft.println("Volume");
                                tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+5+39, 145);
                                tft.println("Vacuum");
                                tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+68+39, 125);
                                tft.println("L");
                                tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+68+39, 145);
                                tft.println("O");
                                tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+68+39, 165);
                                tft.println("H");

                                tft.setTextColor(0xe0e0);
                                tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+83+39, 105);
                                tft.println(volume[i+4]);
                                tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+83+39, 125);
                                tft.println(vacuum_low[i+4]);
                                tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+83+39, 145);
                                tft.println(vacuum_off[i+4]);
                                tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+83+39, 165);
                                tft.println(vacuum_high[i+4]);

                                tft.drawLine(i*((DISPLAY_WIDTH/4)-10)+60+39,134,i*((DISPLAY_WIDTH/4)-10)+60+39,174,FRAME_COLOR);
                                tft.drawLine(i*((DISPLAY_WIDTH/4)-10)+60+39,134,i*((DISPLAY_WIDTH/4)-10)+64+39,134,FRAME_COLOR);
                                tft.drawLine(i*((DISPLAY_WIDTH/4)-10)+60+39,154,i*((DISPLAY_WIDTH/4)-10)+64+39,154,FRAME_COLOR);
                                tft.drawLine(i*((DISPLAY_WIDTH/4)-10)+60+39,174,i*((DISPLAY_WIDTH/4)-10)+64+39,174,FRAME_COLOR);
                            
                                tft.fillRoundRect(i*((DISPLAY_WIDTH/4)-10)+30+39,200,50,25,5,BUTTON_COLOR_1);
                                tft.setTextColor(BUTTON_TEXT_COLOR);
                                tft.setFontScale(TEXT_SIZE_SMALL);
                                tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+40+39, 204);
                                tft.println("EDIT");
                            }

                            tft.setTextColor(TEXT_COLOR_1);
                            tft.setFontScale(TEXT_SIZE_SMALL);
                            if(newprogram1==1)
                            {
                                tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+11+39, 65);
                                tft.println("DMT Removal");
                                tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+20+39, 65);
                                tft.println("Washing 2");
                                tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+26+39, 65);
                                tft.println("Elution");
                            }

                            if(newprogram2==1)
                            {
                                tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+26+39, 65);
                                tft.println("Elution");;
                            }
                            
                        }
                    }

                    tft.fillRect(2,4,100,22,FRAME_COLOR);
                    tft.setTextColor(0xffff);
                    tft.setCursor(4,7);
                    tft.println(tx);
                    tft.setCursor(50,7);
                    tft.println(ty);
                }

                if(group_stage == 2)
                {
                    if(tx>0 && tx<37 && ty>60 && ty<234) //arrow pressed 
                    {
                        tx=0;
                        ty=0;

                        configScreen();
                    }
                    if(tx>69 && tx<119 && ty>200 && ty<225) //edit 5 pressed 
                    {
                        tx=0;
                        ty=0;

                        stage_config = 4;
                        editWindow(5); 
                    }
                    
                    if(tx>179 && tx<229 && ty>200 && ty<225) //edit 6 pressed
                    {
                        tx=0;
                        ty=0;
                        if(stages>5)
                        {
                            stage_config = 5;
                            editWindow(6);
                        }
                    }
                    
                    if(tx>289 && tx<339 && ty>200 && ty<225) //edit 7 pressed
                    {
                        tx=0;
                        ty=0;
                        if(stages>6)
                        {
                            stage_config = 6;
                            editWindow(7);
                        }
                    }

                    if(tx>399 && tx<449 && ty>200 && ty<225) //edit 8 pressed
                    {
                        tx=0;
                        ty=0;
                        if(stages>7)
                        {
                            stage_config = 7;
                            editWindow(8);
                        }
                    }
                }

                if(tx>402 && tx<477 && ty>33 && ty<58) //default pressed 
                {
                    tx=0;
                    ty=0;
                    
                    if(newprogram1==1)
                    {
                        volume[0]=300;
                        volume[1]=300; 
                        volume[2]=380; 
                        volume[3]=300; 
                        volume[4]=300; 
                        volume[5]=300; 
                        volume[6]=200;

                        vacuum_low[0]=30;
                        vacuum_low[1]=30;
                        vacuum_low[2]=30;
                        vacuum_low[3]=30;
                        vacuum_low[4]=30;
                        vacuum_low[5]=30;
                        vacuum_low[6]=30;

                        vacuum_off[0]=0;
                        vacuum_off[1]=0;
                        vacuum_off[2]=0;
                        vacuum_off[3]=0;
                        vacuum_off[4]=0;
                        vacuum_off[5]=0;
                        vacuum_off[6]=30;

                        vacuum_high[0]=15;
                        vacuum_high[1]=15;
                        vacuum_high[2]=15;
                        vacuum_high[3]=15;
                        vacuum_high[4]=15;
                        vacuum_high[5]=15;
                        vacuum_high[6]=15;
                    }

                    if(newprogram2==1)
                    {
                        tft.fillRoundRect(90,35,300,200,5,0xd6d6);
                        tft.fillRoundRect(90,35,300,35,5,0x2525);
                        tft.setTextColor(0xffff);
                        tft.setFontScale(TEXT_SIZE_SMALL);
                        tft.setCursor(194, 44);
                        tft.println("SAMPLE TYPE");
                        tft.setTextColor(0x0000);
                        tft.setCursor(118, 100);
                        tft.println("Please select the sample type !");
                        tft.fillRoundRect(135,189,75,25,5,0x2525);
                        tft.setTextColor(0xffff);
                        tft.setCursor(149, 193);
                        tft.println("Single");
                        tft.fillRoundRect(270,189,75,25,5,0x2525);
                        tft.setTextColor(0xffff);
                        tft.setCursor(289, 193);
                        tft.println("Multi");

                        while(1)
                        {   
                            if(tft.touchDetect())
                            {  
                                tft.touchReadPixel(&tx, &ty);
                                if(tx>135 && tx<210 && ty>189 && ty<214)
                                {
                                    tx=0;
                                    ty=0;
                                    
                                    volume[0]=100;
                                    volume[1]=300; 
                                    volume[2]=300; 
                                    volume[3]=300; 
                                    volume[4]=150; 
  
                                    vacuum_low[0]=10;
                                    vacuum_low[1]=10;
                                    vacuum_low[2]=20;
                                    vacuum_low[3]=10;
                                    vacuum_low[4]=15;
                         
                                    vacuum_off[0]=0;
                                    vacuum_off[1]=0;
                                    vacuum_off[2]=0;
                                    vacuum_off[3]=0;
                                    vacuum_off[4]=0;
                                    
                                    vacuum_high[0]=30;
                                    vacuum_high[1]=30;
                                    vacuum_high[2]=15;
                                    vacuum_high[3]=30;
                                    vacuum_high[4]=20;

                                    configScreen();
                                    break;
                                }

                                if(tx>270 && tx<345 && ty>189 && ty<214)
                                {
                                    tx=0;
                                    ty=0;
                                    
                                    volume[0]=100;
                                    volume[1]=300; 
                                    volume[2]=500; 
                                    volume[3]=500; 
                                    volume[4]=150; 
  
                                    vacuum_low[0]=10;
                                    vacuum_low[1]=10;
                                    vacuum_low[2]=30;
                                    vacuum_low[3]=20;
                                    vacuum_low[4]=15;
                         
                                    vacuum_off[0]=0;
                                    vacuum_off[1]=0;
                                    vacuum_off[2]=0;
                                    vacuum_off[3]=0;
                                    vacuum_off[4]=0;
                                    
                                    vacuum_high[0]=30;
                                    vacuum_high[1]=30;
                                    vacuum_high[2]=20;
                                    vacuum_high[3]=30;
                                    vacuum_high[4]=20;
                                    
                                    configScreen();
                                    break;
                                }
                            }
                        }
                    }

                    configScreen();
                }

                if(tx>146 && tx<212 && ty>24 && ty<65) //samples pressed 
                {
                    tx=0;
                    ty=0;

                    sampleWindow();
                }

                if(tx>2 && tx<77 && ty>244 && ty<269) //back pressed   
                {
                    tx=0;
                    ty=0;
                    programScreen();
                }
                
                if(tx>201 && tx<276 && ty>244 && ty<269) //save pressed   
                {
                    tx=0;
                    ty=0;
                    uint16_t volume1[8], volume2[8], vacuum_off1[8], vacuum_off2[8], vacuum_low1[8], vacuum_low2[8], vacuum_high1[8], vacuum_high2[8];
                    if(newprogram1==1)
                    {
                        for(int ep=0; ep<7; ep++)
                        {
                            if(volume[ep]>255)
                            {
                                volume1[ep] = volume[ep]/10;
                                volume2[ep] = volume[ep] - volume1[ep]*10;
                                EEPROM.update(ep+300,1);
                            }
                            else
                            {
                                volume1[ep] = volume[ep];
                                volume2[ep] = 0;
                                EEPROM.update(ep+300,0);
                            }
                            if(vacuum_off[ep]>255)
                            {
                                vacuum_off1[ep] = vacuum_off[ep]/10;
                                vacuum_off2[ep] = vacuum_off[ep] - vacuum_off1[ep]*10;
                                EEPROM.update(ep+300+15,1);
                            }
                            else
                            {
                                vacuum_off1[ep] = vacuum_off[ep];
                                vacuum_off2[ep] = 0;
                                EEPROM.update(ep+300+15,0);
                            }
                            if(vacuum_low[ep]>255)
                            {
                                vacuum_low1[ep] = vacuum_low[ep]/10;
                                vacuum_low2[ep] = vacuum_low[ep] - vacuum_low1[ep]*10;
                                EEPROM.update(ep+300+30,1);
                            }
                            else
                            {
                                vacuum_low1[ep] = vacuum_low[ep];
                                vacuum_low2[ep] = 0;
                                EEPROM.update(ep+300+30,0);
                            }
                            if(vacuum_high[ep]>255)
                            {
                                vacuum_high1[ep] = vacuum_high[ep]/10;
                                vacuum_high2[ep] = vacuum_high[ep] - vacuum_high1[ep]*10;
                                EEPROM.update(ep+300+45,1);
                            }
                            else
                            {
                                vacuum_high1[ep] = vacuum_high[ep];
                                vacuum_high2[ep] = 0;
                                EEPROM.update(ep+300+45,0);
                            }
                            EEPROM.update(ep+500,volume1[ep]);
                            EEPROM.update(ep+500+15,vacuum_off1[ep]);
                            EEPROM.update(ep+500+30,vacuum_low1[ep]);
                            EEPROM.update(ep+500+45,vacuum_high1[ep]);

                            EEPROM.update(ep+600,volume2[ep]);
                            EEPROM.update(ep+600+15,vacuum_off2[ep]);
                            EEPROM.update(ep+600+30,vacuum_low2[ep]);
                            EEPROM.update(ep+600+45,vacuum_high2[ep]);
                        }
                    }

                    if(newprogram2==1)
                    {
                        for(int ep=0; ep<5; ep++)
                        {
                            if(volume[ep]>255)
                            {
                                volume1[ep] = volume[ep]/10;
                                volume2[ep] = volume[ep] - volume1[ep]*10;
                                EEPROM.update(ep+400,1);
                            }
                            else
                            {
                                volume1[ep] = volume[ep];
                                volume2[ep] = 0;
                                EEPROM.update(ep+400,0);
                            }
                            if(vacuum_off[ep]>255)
                            {
                                vacuum_off1[ep] = vacuum_off[ep]/10;
                                vacuum_off2[ep] = vacuum_off[ep] - vacuum_off1[ep]*10;
                                EEPROM.update(ep+400+15,1);
                            }
                            else
                            {
                                vacuum_off1[ep] = vacuum_off[ep];
                                vacuum_off2[ep] = 0;
                                EEPROM.update(ep+400+15,0);
                            }
                            if(vacuum_low[ep]>255)
                            {
                                vacuum_low1[ep] = vacuum_low[ep]/10;
                                vacuum_low2[ep] = vacuum_low[ep] - vacuum_low1[ep]*10;
                                EEPROM.update(ep+400+30,1);
                            }
                            else
                            {
                                vacuum_low1[ep] = vacuum_low[ep];
                                vacuum_low2[ep] = 0;
                                EEPROM.update(ep+400+30,0);
                            }
                            if(vacuum_high[ep]>255)
                            {
                                vacuum_high1[ep] = vacuum_high[ep]/10;
                                vacuum_high2[ep] = vacuum_high[ep] - vacuum_high1[ep]*10;
                                EEPROM.update(ep+400+45,1);
                            }
                            else
                            {
                                vacuum_high1[ep] = vacuum_high[ep];
                                vacuum_high2[ep] = 0;
                                EEPROM.update(ep+400+45,0);
                            }
                            
                            EEPROM.update(ep+700,volume1[ep]);
                            EEPROM.update(ep+700+15,vacuum_off1[ep]);
                            EEPROM.update(ep+700+30,vacuum_low1[ep]);
                            EEPROM.update(ep+700+45,vacuum_high1[ep]);

                            EEPROM.update(ep+800,volume2[ep]);
                            EEPROM.update(ep+800+15,vacuum_off2[ep]);
                            EEPROM.update(ep+800+30,vacuum_low2[ep]);
                            EEPROM.update(ep+800+45,vacuum_high2[ep]);
                        }
                    }

                    tft.fillRoundRect(90,35,300,200,5,0xd6d6);
                    tft.fillRoundRect(90,35,300,35,5,0x2525);
                    tft.setTextColor(0xffff);
                    tft.setFontScale(TEXT_SIZE_SMALL);
                    tft.setCursor(220, 44);
                    tft.println("SAVE");
                    tft.setTextColor(0x0000);
                    tft.setCursor(183, 103);
                    tft.println("SAVE COMPLETE !");
                    tft.fillRoundRect(200,189,75,25,5,0x2525);
                    tft.setTextColor(0xffff);
                    tft.setCursor(230, 193);
                    tft.println("OK");
                    while(1)
                    {
                        if(tft.touchDetect())
                        {  
                            tft.touchReadPixel(&tx, &ty);
                            if(tx>200 && tx<275 && ty>189 && ty<214)
                            {
                                configScreen();
                                break;
                            }
                        }     
                    }

                }

                if(tx>402 && tx<477 && ty>244 && ty<269) //run pressed 
                {
                    tx=0;
                    ty=0;
                    
                    if(newprogram1==1)
                    {
                        for(int i=0; i<7; i++)
                        {   
                            if(i!=6)
                            {
                                m[i] = int((vacuum_low[i] + vacuum_off[i] + vacuum_high[i])/60);
                            }
                            else
                            {
                                m[i] = int((vacuum_low[i] + vacuum_off[i] + vacuum_high[i] + 5)/60);
                            }
                            if(m[i]>0)
                            {
                                if(i!=6)
                                {
                                    s[i] = (vacuum_low[i] + vacuum_off[i] + vacuum_high[i])-(m[i]*60);
                                }
                                else
                                {
                                    s[i] = (vacuum_low[i] + vacuum_off[i] + vacuum_high[i] + 5)-(m[i]*60);
                                }
                            }
                            else
                            {
                                s[i] = (vacuum_low[i] + vacuum_off[i] + vacuum_high[i]);
                            }
                        }

                        chemical[0] = 1;
                        chemical[1] = 2;
                        chemical[2] = 0;
                        chemical[3] = 2;
                        chemical[4] = 3;
                        chemical[5] = 2;
                        chemical[6] = 1;
                    }
                    if(newprogram2==1)
                    {
                        for(int i=0; i<5; i++)
                        {
                            m[i] = int((vacuum_low[i] + vacuum_off[i] + vacuum_high[i])/60);
                            if(m[i]>0)
                                s[i] = (vacuum_low[i] + vacuum_off[i] + vacuum_high[i])-(m[i]*60);
                            else
                                s[i] = (vacuum_low[i] + vacuum_off[i] + vacuum_high[i]);
                        }

                        chemical[0] = 1;
                        chemical[1] = 2;
                        chemical[2] = 0;
                        chemical[3] = 2;
                        chemical[4] = 3;
                    }

                    group_stage = 1;
                    run_value = 1;
                    runScreen();
                    delayPress = 1;
                }
            }

            if(screen==3)    //edit window
            {
                if(tx>116 && tx<200 && ty>42 && ty<73)   //volume pressed 
                {      
                    tx=0;
                    ty=0;

                    tft.drawRect(114,40,83,35,FRAME_COLOR);
                    tft.drawRect(113,39,85,37,FRAME_COLOR);
                    tft.drawRect(112,38,87,39,FRAME_COLOR);

                    tft.drawRect(114,134,83,35,BACK_GROUND_COLOR);
                    tft.drawRect(113,133,85,37,BACK_GROUND_COLOR);
                    tft.drawRect(112,132,87,39,BACK_GROUND_COLOR);

                    tft.drawRect(114,181,83,35,BACK_GROUND_COLOR);
                    tft.drawRect(113,180,85,37,BACK_GROUND_COLOR);
                    tft.drawRect(112,179,87,39,BACK_GROUND_COLOR);

                    tft.drawRect(114,228,83,35,BACK_GROUND_COLOR);
                    tft.drawRect(113,227,85,37,BACK_GROUND_COLOR);
                    tft.drawRect(112,226,87,39,BACK_GROUND_COLOR);

                    n=0;
                }
                if(tx>116 && tx<200 && ty>136 && ty<167)   //vacuum low pressed
                {
                    tx=0;
                    ty=0;

                    tft.drawRect(114,40,83,35,BACK_GROUND_COLOR);
                    tft.drawRect(113,39,85,37,BACK_GROUND_COLOR);
                    tft.drawRect(112,38,87,39,BACK_GROUND_COLOR);

                    tft.drawRect(114,134,83,35,FRAME_COLOR);
                    tft.drawRect(113,133,85,37,FRAME_COLOR);
                    tft.drawRect(112,132,87,39,FRAME_COLOR);

                    tft.drawRect(114,181,83,35,BACK_GROUND_COLOR);
                    tft.drawRect(113,180,85,37,BACK_GROUND_COLOR);
                    tft.drawRect(112,179,87,39,BACK_GROUND_COLOR);

                    tft.drawRect(114,228,83,35,BACK_GROUND_COLOR);
                    tft.drawRect(113,227,85,37,BACK_GROUND_COLOR);
                    tft.drawRect(112,226,87,39,BACK_GROUND_COLOR);

                    n=1;
                }
                if(tx>116 && tx<200 && ty>183 && ty<214)   //vacuum off pressed
                {
                    tx=0;
                    ty=0;

                    tft.drawRect(114,40,83,35,BACK_GROUND_COLOR);
                    tft.drawRect(113,39,85,37,BACK_GROUND_COLOR);
                    tft.drawRect(112,38,87,39,BACK_GROUND_COLOR);

                    tft.drawRect(114,134,83,35,BACK_GROUND_COLOR);
                    tft.drawRect(113,133,85,37,BACK_GROUND_COLOR);
                    tft.drawRect(112,132,87,39,BACK_GROUND_COLOR);

                    tft.drawRect(114,181,83,35,FRAME_COLOR);
                    tft.drawRect(113,180,85,37,FRAME_COLOR);
                    tft.drawRect(112,179,87,39,FRAME_COLOR);

                    tft.drawRect(114,228,83,35,BACK_GROUND_COLOR);
                    tft.drawRect(113,227,85,37,BACK_GROUND_COLOR);
                    tft.drawRect(112,226,87,39,BACK_GROUND_COLOR);

                    n=2;
                }
                if(tx>116 && tx<200 && ty>230 && ty<261)   //vacuum high low pressed
                {
                    tx=0;
                    ty=0;

                    tft.drawRect(114,40,83,35,BACK_GROUND_COLOR);
                    tft.drawRect(113,39,85,37,BACK_GROUND_COLOR);
                    tft.drawRect(112,38,87,39,BACK_GROUND_COLOR);

                    tft.drawRect(114,134,83,35,BACK_GROUND_COLOR);
                    tft.drawRect(113,133,85,37,BACK_GROUND_COLOR);
                    tft.drawRect(112,132,87,39,BACK_GROUND_COLOR);

                    tft.drawRect(114,181,83,35,BACK_GROUND_COLOR);
                    tft.drawRect(113,180,85,37,BACK_GROUND_COLOR);
                    tft.drawRect(112,179,87,39,BACK_GROUND_COLOR);

                    tft.drawRect(114,228,83,35,FRAME_COLOR);
                    tft.drawRect(113,227,85,37,FRAME_COLOR);
                    tft.drawRect(112,226,87,39,FRAME_COLOR);

                    n=3;
                }

                if(ty>51 && ty<96)  //key 7 8 9 pressed 
                {
                    for(int i=0; i<3; i++)  
                    {
                        if(tx>((i*52)+231) && tx<((i*52)+256))
                        {
                            tx=0;
                            ty=0;

                            if(key[n]<100)     key[n]=key[n]*10+i+7;            
                        }
                    }      
                }

                if(ty>103 && ty<148)   //key 4 5 6 pressed
                {
                    for(int i=0; i<3; i++)
                    {
                        if(tx>((i*52)+231) && tx<((i*52)+256))
                        {
                            tx=0;
                            ty=0;

                            if(key[n]<100)     key[n]=key[n]*10+i+4;   
                        }
                    }      
                }

                if(ty>155 && ty<200)   //key 1 2 3 pressed
                {
                    for(int i=0; i<3; i++)
                    {
                        if(tx>((i*52)+231) && tx<((i*52)+256))
                        {
                            tx=0;
                            ty=0;

                            if(key[n]<100)    key[n]=key[n]*10+i+1;   
                        }
                    }      
                }

                if(ty>207 && ty<252)
                {
                    if(tx>231 && tx<256)   //key 0 pressed 
                    {
                        tx=0;
                        ty=0;

                        if(key[n]<100)     key[n]=key[n]*10;   
                    }
                    if(tx>283 && tx<308)   //key c pressed 
                    {
                        tx=0;
                        ty=0;

                        key[n]=0;  
                    }
                    if(tx>335 && tx<360)   //key << pressed 
                    {
                        tx=0;
                        ty=0;

                        key[n]=(key[n]-(key[n]%10))/10;   
                    }
                }

                tft.setTextColor(RA8875_BLACK,RA8875_WHITE);
                tft.setFontScale(1);
                if(n==0)
                {
                    tft.setCursor(116,42);
                    sprintf(u1,"%5.0d",key[n]);    
                    tft.print(u1);
                }
                if(n==1)
                {
                    tft.setCursor(116,136);
                    sprintf(u2,"%5.0d",key[n]);    
                    tft.print(u2);
                }
                if(n==2)
                {
                    tft.setCursor(116,183);
                    sprintf(u3,"%5.0d",key[n]);    
                    tft.print(u3);
                }
                if(n==3)
                {
                    tft.setCursor(116,230);
                    sprintf(u4,"%5.0d",key[n]);    
                    tft.print(u4);
                }    
                
                if(tx>403 && tx<482 && ty>112 && ty<140)  //ok pressed 
                {
                    tx=0;
                    ty=0;

                    for(int o=0;o<stages;o++)
                    {
                        if(stage_config==o)
                        {
                            volume[o] = key[0];
                            vacuum_low[o] = key[1];
                            vacuum_off[o] = key[2];
                            vacuum_high[o] = key[3];
                        }         
                    }
                    configScreen();           
                }

                if(tx>403 && tx<482 && ty>165 && ty<195) //cancel pressed
                {
                    tx=0;
                    ty=0;

                    configScreen();
                }
            }

            if(screen==4)   // samples window
            {
                if(ty>51 && ty<96)  //key 7 8 9 pressed 
                {
                    for(int o=0; o<3; o++)  
                    {
                        if(tx>((o*52)+231) && tx<((o*52)+256))
                        {   
                            tx=0;
                            ty=0;

                            if(key[4]<=9)     key[4]=key[4]*10+o+7;            
                        }
                    }      
                }

                if(ty>103 && ty<148)   //key 4 5 6 pressed
                {
                    for(int o=0; o<3; o++)
                    {
                        if(tx>((o*52)+231) && tx<((o*52)+256))
                        {
                            tx=0;
                            ty=0;

                            if(key[4]<=9)     key[4]=key[4]*10+o+4;   
                        }
                    }      
                }

                if(ty>155 && ty<200)   //key 1 2 3 pressed
                {
                    for(int o=0; o<3; o++)
                    {
                        if(tx>((o*52)+231) && tx<((o*52)+256))
                        {
                            tx=0;
                            ty=0;

                            if(key[4]<=9)    key[4]=key[4]*10+o+1;   
                        }
                    }      
                }

                if(ty>207 && ty<252)
                {
                    if(tx>231 && tx<256)   //key 0 pressed 
                    {
                        tx=0;
                        ty=0;

                        if(key[4]<=9)     key[4]=key[4]*10;   
                    }
                    if(tx>283 && tx<308)   //key c pressed 
                    {
                        tx=0;
                        ty=0;

                        key[4]=0;  
                    }
                    if(tx>335 && tx<360)   //key << pressed 
                    {
                        tx=0;
                        ty=0;

                        key[4]=(key[4]-(key[4]%10))/10;   
                    }
                }

                tft.setTextColor(RA8875_BLACK,RA8875_WHITE);
                tft.setFontScale(1);
                tft.setCursor(69,82);
                sprintf(u_sample,"%5.0d",key[4]);    
                tft.print(u_sample);

                if(tx>403 && tx<482 && ty>112 && ty<140)  //ok pressed 
                {
                    tx=0;
                    ty=0;

                    samples = key[4];

                    configScreen();            
                }

                if(tx>403 && tx<482 && ty>165 && ty<195) //cancel pressed
                {
                    tx=0;
                    ty=0;

                    configScreen();
                }
            }

            if(screen==5) //run 
            {
                if(putchemin==0 && completed==0)
                {
                    if(group_stage==1)
                    {
                        if(tx>442 && tx<475 && ty>33 && ty<239)
                        {
                            tx=0;
                            ty=0;

                            group_stage = 2;
                            tft.fillRect(0,33,DISPLAY_WIDTH,DISPLAY_HEIGHT-66,BACK_GROUND_COLOR);
                            uint8_t run_count;
                            if(newprogram1==1)
                                run_count=3;
                            if(newprogram2==1)
                                run_count=1;
                            for(int y=0; y<run_count;y++)
                            {   
                                tft.drawRect(y*((DISPLAY_WIDTH/4)-10)+39,33,(DISPLAY_WIDTH/4)-10,DISPLAY_HEIGHT-66,FRAME_COLOR);
                                tft.fillRect(y*((DISPLAY_WIDTH/4)-10)+2+39,35,(DISPLAY_WIDTH/4)-14,27,FRAME_COLOR);

                                if(y<3)
                                {
                                    tft.drawCircle(y*((DISPLAY_WIDTH/4)-10)+55+39,177,40,FRAME_COLOR);
                                    tft.drawCircle(y*((DISPLAY_WIDTH/4)-10)+55+39,177,41,FRAME_COLOR);
                                    tft.drawCircle(y*((DISPLAY_WIDTH/4)-10)+55+39,177,42,FRAME_COLOR);
                                    
                                    tft.setFontScale(TEXT_SIZE_SMALL);
                                    //tft.setTextColor(0xffff);
                                    tft.setCursor(y*((DISPLAY_WIDTH/4)-10)+27+39, 169);
                                    tft.println("WAITING");

                                    tft.setFontScale(TEXT_SIZE_MEDIUM);
                                    tft.setTextColor(0xe0e0);
                                    if(m[y+4]<10)
                                    {  
                                        tft.setCursor(y*((DISPLAY_WIDTH/4)-10)+18+39, 76);
                                        tft.println(0);
                                        tft.setCursor(y*((DISPLAY_WIDTH/4)-10)+33+39, 76);
                                        tft.println(m[y+4]);
                                    }
                                    else
                                    {
                                        tft.setCursor(y*((DISPLAY_WIDTH/4)-10)+18+39, 76);
                                        tft.println(m[y+4]);
                                    }

                                    tft.setCursor(y*((DISPLAY_WIDTH/4)-10)+48+39, 76);
                                    tft.println(":");

                                    if(s[y+4]<10)
                                    {
                                        tft.setCursor(y*((DISPLAY_WIDTH/4)-10)+63+39, 76);
                                        tft.println(0);
                                        tft.setCursor(y*((DISPLAY_WIDTH/4)-10)+78+39, 76);
                                        tft.println(s[y+4]);
                                    }
                                    else
                                    {
                                        tft.setCursor(y*((DISPLAY_WIDTH/4)-10)+63+39, 76);
                                        tft.println(s[y+4]);
                                    }
                                }
                            }

                            tft.fillRect(0,33,37,DISPLAY_HEIGHT-66,FRAME_COLOR);
                            tft.fillTriangle(28,120,28,150,8,135,0xffff);
                            
                            tft.setTextColor(TEXT_COLOR_1);
                            tft.setFontScale(TEXT_SIZE_SMALL);
                            if(newprogram1==1)
                            {
                                tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+11+39, 40);
                                tft.println("DMT Removal");
                                tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+31+39, 40);
                                tft.println("Wash 2");
                                tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+28+39, 40);
                                tft.println("Elution");
                            }
                            if(newprogram2==1)
                            {
                                tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+26+39, 40);
                                tft.println("Elution");;
                            }
                            runScreenUpdate(run_scr);
                            
                        }
                    }

                    if(group_stage==2)
                    {
                        if(tx>0 && tx<37 && ty>33 && ty<239)
                        {
                            tx=0;
                            ty=0;

                            group_stage = 1;
                            runScreen();
                            runScreenUpdate(run_scr);
                        }
                    }

                    // if(tx>2 && tx<77 && ty>244 && ty<269) //pause pressed   
                    // {
                    //     delayPress = 1;
                    //     tx=0;
                    //     ty=0;
                        
                    //     if(pause_click==0) 
                    //     {
                            
                    //         digitalWrite(X_ENABLE_PIN, HIGH);
                    //         digitalWrite(Y_ENABLE_PIN, HIGH);
                    //         digitalWrite(PRESSURE_PIN, LOW);
                    //         analogWrite(PWM_PIN,0);

                    //         tft.fillRoundRect(2,DISPLAY_HEIGHT-30+2,75,25,5,BUTTON_COLOR_1);
                    //         tft.setTextColor(BUTTON_TEXT_COLOR);
                    //         tft.setFontScale(TEXT_SIZE_SMALL);
                    //         tft.setCursor(20, DISPLAY_HEIGHT-30+7);
                    //         tft.println("RESUME");
                            
                    //         pause_click = 1;
                    //         while(pause_click == 1)
                    //         {
                    //             if(tft.touchDetect())
                    //             {
                    //                 delayPress = 1;
                    //                 tft.touchReadPixel(&tx, &ty); 
                    //                 if(tx>2 && tx<77 && ty>244 && ty<269)
                    //                 {
                    //                     tx=0;
                    //                     ty=0;

                    //                     digitalWrite(X_ENABLE_PIN,LOW);
                    //                     digitalWrite(Y_ENABLE_PIN,LOW);
                    //                     digitalWrite(X_ENABLE_PIN, LOW);
                    //                     digitalWrite(Y_ENABLE_PIN, LOW);
                    //                     if(vaccume_state==0)
                    //                     {
                    //                         digitalWrite(PRESSURE_PIN, LOW);
                    //                         analogWrite(PWM_PIN,0);
                    //                     }
                    //                     if(vaccume_state==1)
                    //                     {
                    //                         digitalWrite(PRESSURE_PIN, HIGH);
                    //                         analogWrite(PWM_PIN,105);
                    //                     }
                    //                     if(vaccume_state==2)
                    //                     {
                    //                         digitalWrite(PRESSURE_PIN, LOW);
                    //                         analogWrite(PWM_PIN, pulse_set);
                    //                     }
                                        
                    //                     tft.fillRoundRect(2,DISPLAY_HEIGHT-30+2,75,25,5,BUTTON_COLOR_1);
                    //                     tft.setTextColor(BUTTON_TEXT_COLOR);
                    //                     tft.setFontScale(TEXT_SIZE_SMALL);
                    //                     tft.setCursor(20, DISPLAY_HEIGHT-30+7);
                    //                     tft.println("PAUSE");
                    //                     pause_click = 0;                                    
                    //                 }
                    //             }
                    //         }      
                    //     }    
                    // }
                    
                    if(tx>199 && tx<274 && ty>244 && ty<269) //stop pressed 
                    {
                        tx=0;
                        ty=0;

                        tft.fillRoundRect(90,35,300,200,5,0xd6d6);
                        tft.fillRoundRect(90,35,300,35,5,0x2525);
                        tft.setTextColor(0xffff);
                        tft.setFontScale(TEXT_SIZE_SMALL);
                        tft.setCursor(192, 44);
                        tft.println("STOP PROGRAM");
                        tft.setTextColor(0x0000);
                        tft.setCursor(93, 95);
                        tft.println("> Press \"Resume\" to countinue");
                        tft.setCursor(93, 118);
                        tft.println("> Press \"Exit\" to back to main screen");
                        tft.fillRoundRect(135,189,75,25,5,0x2525);
                        tft.setTextColor(0xffff);
                        tft.setCursor(150, 193);
                        tft.println("Resume");
                        tft.fillRoundRect(270,189,75,25,5,0x2525);
                        tft.setTextColor(0xffff);
                        tft.setCursor(293, 193);
                        tft.println("Exit");

                        digitalWrite(X_ENABLE_PIN, HIGH);
                        digitalWrite(Y_ENABLE_PIN, HIGH);
                        digitalWrite(PRESSURE_PIN, LOW);
                        analogWrite(PWM_PIN,0);

                        while(1)
                        {   
                            if(tft.touchDetect())
                            {  
                                tft.touchReadPixel(&tx, &ty);
                                if(tx>135 && tx<210 && ty>189 && ty<214)
                                {
                                    tx=0;
                                    ty=0;

                                    digitalWrite(X_ENABLE_PIN, LOW);
                                    digitalWrite(Y_ENABLE_PIN, LOW);
                                    if(vaccume_state==0)
                                    {
                                        digitalWrite(PRESSURE_PIN, LOW);
                                        analogWrite(PWM_PIN,0);
                                    }
                                    if(vaccume_state==1)
                                    {
                                        digitalWrite(PRESSURE_PIN, HIGH);
                                        analogWrite(PWM_PIN,pulse_low_set);
                                    }
                                    if(vaccume_state==2)
                                    {
                                        digitalWrite(PRESSURE_PIN, LOW);
                                        analogWrite(PWM_PIN, pulse_high_set);
                                    }
                                    
                                    if(run_scr>4)
                                    {
                                        group_stage = 2;
                                        tft.fillRect(0,33,DISPLAY_WIDTH,DISPLAY_HEIGHT-66,BACK_GROUND_COLOR);
                                        uint8_t run_count;
                                        if(newprogram1==1)
                                            run_count=3;
                                        if(newprogram2==1)
                                            run_count=1;
                                        for(int i=0; i<run_count;i++)
                                        {   
                                            tft.drawRect(i*((DISPLAY_WIDTH/4)-10)+39,33,(DISPLAY_WIDTH/4)-10,DISPLAY_HEIGHT-66,FRAME_COLOR);
                                            tft.fillRect(i*((DISPLAY_WIDTH/4)-10)+2+39,35,(DISPLAY_WIDTH/4)-14,27,FRAME_COLOR);

                                            if(i<3)
                                            {
                                                tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55+39,177,40,FRAME_COLOR);
                                                tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55+39,177,41,FRAME_COLOR);
                                                tft.drawCircle(i*((DISPLAY_WIDTH/4)-10)+55+39,177,42,FRAME_COLOR);
                                                
                                                tft.setFontScale(TEXT_SIZE_SMALL);
                                                //tft.setTextColor(0xffff);
                                                tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+27+39, 169);
                                                tft.println("WAITING");

                                                tft.setFontScale(TEXT_SIZE_MEDIUM);
                                                tft.setTextColor(0xe0e0);
                                                if(m[i+4]<10)
                                                {  
                                                    tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+18+39, 76);
                                                    tft.println(0);
                                                    tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+33+39, 76);
                                                    tft.println(m[i+4]);
                                                }
                                                else
                                                {
                                                    tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+18+39, 76);
                                                    tft.println(m[i+4]);
                                                }

                                                tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+48+39, 76);
                                                tft.println(":");

                                                if(s[i+4]<10)
                                                {
                                                    tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+63+39, 76);
                                                    tft.println(0);
                                                    tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+78+39, 76);
                                                    tft.println(s[i+4]);
                                                }
                                                else
                                                {
                                                    tft.setCursor(i*((DISPLAY_WIDTH/4)-10)+63+39, 76);
                                                    tft.println(s[i+4]);
                                                }
                                            }
                                        }
                                        tft.fillRect(0,33,37,DISPLAY_HEIGHT-66,FRAME_COLOR);
                                        tft.fillTriangle(28,120,28,150,8,135,0xffff);
                                        
                                        tft.setTextColor(TEXT_COLOR_1);
                                        tft.setFontScale(TEXT_SIZE_SMALL);
                                        if(newprogram1==1)
                                        {
                                            tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+11+39, 40);
                                            tft.println("DMT Removal");
                                            tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+31+39, 40);
                                            tft.println("Wash 2");
                                            tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+28+39, 40);
                                            tft.println("Elution");
                                        }
                                        if(newprogram2==1)
                                        {
                                            tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+26+39, 40);
                                            tft.println("Elution");;
                                        }
                                        runScreenUpdate(run_scr);
                                    }
                                    else
                                    {
                                        group_stage = 1;
                                        runScreen();
                                        runScreenUpdate(run_scr);
                                    }
                                    
                                   
                                    break;
                                }

                                if(tx>270 && tx<345 && ty>189 && ty<214)
                                {
                                    tx=0;
                                    ty=0;
                                    resetFunc();
                                    break;
                                }
                            }     
                        }
                    }    
                }
            }

            //ispress = 1;
        }
    }
}
////////////////////  PRESS - END  /////////////////////

/////////////////// STEPPER - START ////////////////////
void home(char *AXIS)
{ 
    home_value=1;
    digitalWrite(X_DIR_PIN,LOW);
    digitalWrite(Y_DIR_PIN,LOW);
    digitalWrite(X_ENABLE_PIN,LOW);
    digitalWrite(Y_ENABLE_PIN,LOW);

    if(AXIS=="X") y_limit = 1;
    if(AXIS=="Y") x_limit = 1;
    if(AXIS=="XY")
    {
        y_limit == 0;
        x_limit == 0;
    }

    while(x_limit==0 || y_limit==0)
    {
        if(x_limit==0)  digitalWrite(X_STEP_PIN,HIGH); 
        if(y_limit==0)  digitalWrite(Y_STEP_PIN,HIGH);
        delayMicroseconds(10);
        if(x_limit==0)  digitalWrite(X_STEP_PIN,LOW); 
        if(y_limit==0)  digitalWrite(Y_STEP_PIN,LOW);
        delayMicroseconds(30);
        
        if(digitalRead(Y_SW_PIN)==LOW)
        {
            delay(500);
            digitalWrite(Y_DIR_PIN,HIGH);
            for(int k=0;k<=3600;k++)
            {
                digitalWrite(Y_STEP_PIN,HIGH);
                delayMicroseconds(10);
                digitalWrite(Y_STEP_PIN,LOW);
                delayMicroseconds(30);
            }
            y_limit = 1;
        }

        if(digitalRead(X_SW_PIN)==LOW)
        { 
            delay(500);
            digitalWrite(X_DIR_PIN,HIGH);
            for(int f=0;f<=3600;f++)
            {
                digitalWrite(X_STEP_PIN,HIGH);
                delayMicroseconds(10);
                digitalWrite(X_STEP_PIN,LOW);
                delayMicroseconds(30);
            }
            x_limit = 1; 
        }
    }
    digitalWrite(X_ENABLE_PIN, HIGH);
    digitalWrite(Y_ENABLE_PIN, HIGH);
    home_value = 0;
    x_limit = 0;
    y_limit = 0;
}

void run()
{
    uint32_t odd_samples, group4_samples, column1, column2;
    odd_samples = samples%4; 
    if(odd_samples!=0)      group4_samples = int(samples/4) + 1;
    else                    group4_samples = samples/4;
    
    int tmp = round(((float)samples/4)+0.25);
    if(newprogram1==1)
    {
        pulse_low_set = 150;
        for(int p=1;p<=24;p++)
        {
            if(p>tmp)  
                pulse_high_set -= 6;                    
        }
    }

    if(newprogram2==1)
    {
        pulse_low_set = 240;
        for(int p=1;p<=24;p++)
        {
            if(p>tmp)  
                pulse_high_set -= 2;                    
        }
    }

    if(pulse_high_set < pulse_low_set)
    {
        pulse_high_set = pulse_low_set;
    }

    home("XY"); // return home
    
    for(uint8_t r=0; r<stages; r++)
    {
        run_scr = r+1;
        if(run_scr==5)
        {
            run_switch_screen = 1;
        }
        runScreenUpdate(run_scr);
        // First move
        if(firstmove==0)
        {
            if(chemical[r]==1)
                move(25,226,"RIGHT","FORWARD");
            else if(chemical[r]==2)
                move(25+9,226,"RIGHT","FORWARD");
            else if(chemical[r]==3)
                move(25+18,226,"RIGHT","FORWARD");

            firstmove = 1;
        }
        else
        {
            if(chemical[r]==1)
                move(25,0,"RIGHT","FORWARD");
            else if(chemical[r]==2)
                move(25+9,0,"RIGHT","FORWARD");
            else if(chemical[r]==3)
                move(25+18,0,"RIGHT","FORWARD");
        }

        if(chemical[r]==0)
        {
            home("Y");
            putchemin = 1;
            tft.fillRoundRect(90,35,300,200,5,0xd6d6);
            tft.fillRoundRect(90,35,300,35,5,0x2525);
            tft.setTextColor(0xffff);
            tft.setFontScale(TEXT_SIZE_SMALL);
            tft.setCursor(179, 44);
            tft.println("PUT CHEMICAL IN");
            tft.setTextColor(0x0000);
            tft.setCursor(98, 103);
            tft.println("Press \"CONTINUE\" when you are done !");
            tft.fillRoundRect(200,189,75,25,5,0x2525);
            tft.setTextColor(0xffff);
            tft.setCursor(206, 193);
            tft.println("CONTINUE");
            while(putchemin==1)
            {
                if(tft.touchDetect())
                {  
                    tft.touchReadPixel(&tx, &ty);
                    if(tx>200 && tx<275 && ty>189 && ty<214)
                    {
                        if(run_scr>4)
                        {
                            group_stage = 2;
                            run_switch_screen = 1;
                        }
                        else
                        {
                            group_stage = 1;
                        }
                        runScreen();
                        runScreenUpdate(run_scr);
                        move(0,226,"NONE","FORWARD");
                        putchemin = 0;
                        delay(wait_time);
                        break;
                    }
                }     
            }
        }

        delay(wait_time);

        if(chemical[r]!=0)
        {
            if(samples<4) 
            output(chemical[r], odd_samples, volume[r]);
            else
            output(chemical[r], 4, volume[r]);
            
            if(group4_samples%2 != 0)  // Nếu số cột hàng trên = số cột hàng dưới
            {
                column1 = round((group4_samples/2)+0.5);
                column2 = column1 - 1;
                
                if(column1-1 > 0)
                {
                    digitalWrite(X_ENABLE_PIN,LOW);
                    digitalWrite(Y_ENABLE_PIN,LOW);
                    digitalWrite(X_DIR_PIN,HIGH);
                    digitalWrite(Y_DIR_PIN,HIGH);

                    for(uint8_t j=0;j<(column1-1);j++)
                    {
                        move(9,0,"RIGHT","NONE");
                        delay(wait_time);
                        if(j==column1-2)    
                            output(chemical[r],odd_samples,volume[r]);
                        else                   
                            output(chemical[r],4,volume[r]); 
                    }

                    move(9,36,"LEFT","FORWARD");
                    delay(wait_time);

                    output(chemical[r],4,volume[r]);
                    for(uint8_t j=0;j<(column2-1);j++)
                    {
                        move(9,0,"LEFT","NONE");
                        delay(wait_time);
                        output(chemical[r],4,volume[r]);
                    }    
                }
            }

            if(group4_samples%2==0 && samples>4) // Nếu số cột hàng trên khác số cột hàng dưới
            {
                column1 = group4_samples/2;
                column2 = column1;

                for(int j=0;j<(column1-1);j++)
                {
                    move(9,0,"RIGHT","NONE");
                    delay(wait_time);
                    output(chemical[r],4,volume[r]);
                }

                move(0,36,"NONE","FORWARD");
                delay(wait_time);

                if(odd_samples!=0)
                    output(chemical[r],odd_samples,volume[r]);
                else
                    output(chemical[r],4,volume[r]);

                for(int j=0;j<(column2-1);j++)
                {
                    move(9,0,"LEFT","NONE");
                    delay(wait_time);
                    output(chemical[r],4,volume[r]);
                }
            }
        }

        if(chemical[r]==1)
        {
            if(samples>4)   
                move(25,36,"LEFT","REVERSE");
            else           
                move(25,0,"LEFT","NONE");
        }
        if(chemical[r]==2)
        {
            if(samples>4)   
                move(25+9,36,"LEFT","REVERSE");
            else           
                move(25+9,0,"LEFT","NONE");
        }
        if(chemical[r]==3)
        {
            if(samples>4)   
                move(25+9*2,36,"LEFT","REVERSE");
            else           
                move(25+9*2,0,"LEFT","NONE");
        }

        run_time = r+1;

        if(r==6)
        {
            analogWrite(PWM_PIN,105);
            digitalWrite(PRESSURE_PIN,HIGH);
            my_delay(5);
        }
        analogWrite(PWM_PIN,0);
        digitalWrite(PRESSURE_PIN,LOW);
        my_delay(vacuum_off[r]);
        vaccume_state = 1;
        analogWrite(PWM_PIN,pulse_low_set);
        digitalWrite(PRESSURE_PIN,HIGH);
        my_delay(vacuum_low[r]);
        vaccume_state = 2;
        analogWrite(PWM_PIN,pulse_high_set);
        digitalWrite(PRESSURE_PIN,LOW);
        my_delay(vacuum_high[r]);
        vaccume_state = 0;
        analogWrite(PWM_PIN,0);

        run_time = 0;

        if(((r==2 || r==5)&&newprogram1==1) || ((r==3)&&newprogram2==1))
        {
            home("Y");
            putchemin = 1;
            tft.fillRoundRect(90,35,300,200,5,0xd6d6);
            tft.fillRoundRect(90,35,300,35,5,0x2525);
            tft.setTextColor(0xffff);
            tft.setFontScale(TEXT_SIZE_SMALL);
            tft.setCursor(179, 44);
            tft.println("...................");
            tft.setTextColor(0x0000);
            tft.setCursor(98, 103);
            tft.println("Press \"CONTINUE\" when you are done !");
            tft.fillRoundRect(200,189,75,25,5,0x2525);
            tft.setTextColor(0xffff);
            tft.setCursor(206, 193);
            tft.println("CONTINUE");
            while(putchemin==1)
            {
                if(tft.touchDetect())
                {  
                    tft.touchReadPixel(&tx, &ty);
                    if(tx>200 && tx<275 && ty>189 && ty<214)
                    {
                        if(run_scr>=4)
                        {
                            group_stage = 2;
                            tft.fillRect(0,33,DISPLAY_WIDTH,DISPLAY_HEIGHT-66,BACK_GROUND_COLOR);
                            uint8_t run_count;
                            if(newprogram1==1)
                                run_count=3;
                            if(newprogram2==1)
                                run_count=1;
                            for(int y=0; y<run_count;y++)
                            {   
                                tft.drawRect(y*((DISPLAY_WIDTH/4)-10)+39,33,(DISPLAY_WIDTH/4)-10,DISPLAY_HEIGHT-66,FRAME_COLOR);
                                tft.fillRect(y*((DISPLAY_WIDTH/4)-10)+2+39,35,(DISPLAY_WIDTH/4)-14,27,FRAME_COLOR);

                                if(y<3)
                                {
                                    tft.drawCircle(y*((DISPLAY_WIDTH/4)-10)+55+39,177,40,FRAME_COLOR);
                                    tft.drawCircle(y*((DISPLAY_WIDTH/4)-10)+55+39,177,41,FRAME_COLOR);
                                    tft.drawCircle(y*((DISPLAY_WIDTH/4)-10)+55+39,177,42,FRAME_COLOR);
                                    
                                    tft.setFontScale(TEXT_SIZE_SMALL);
                                    //tft.setTextColor(0xffff);
                                    tft.setCursor(y*((DISPLAY_WIDTH/4)-10)+27+39, 169);
                                    tft.println("WAITING");

                                    tft.setFontScale(TEXT_SIZE_MEDIUM);
                                    tft.setTextColor(0xe0e0);
                                    if(m[y+4]<10)
                                    {  
                                        tft.setCursor(y*((DISPLAY_WIDTH/4)-10)+18+39, 76);
                                        tft.println(0);
                                        tft.setCursor(y*((DISPLAY_WIDTH/4)-10)+33+39, 76);
                                        tft.println(m[y+4]);
                                    }
                                    else
                                    {
                                        tft.setCursor(y*((DISPLAY_WIDTH/4)-10)+18+39, 76);
                                        tft.println(m[y+4]);
                                    }

                                    tft.setCursor(y*((DISPLAY_WIDTH/4)-10)+48+39, 76);
                                    tft.println(":");

                                    if(s[y+4]<10)
                                    {
                                        tft.setCursor(y*((DISPLAY_WIDTH/4)-10)+63+39, 76);
                                        tft.println(0);
                                        tft.setCursor(y*((DISPLAY_WIDTH/4)-10)+78+39, 76);
                                        tft.println(s[y+4]);
                                    }
                                    else
                                    {
                                        tft.setCursor(y*((DISPLAY_WIDTH/4)-10)+63+39, 76);
                                        tft.println(s[y+4]);
                                    }
                                }
                            }

                            tft.fillRect(0,33,37,DISPLAY_HEIGHT-66,FRAME_COLOR);
                            tft.fillTriangle(28,120,28,150,8,135,0xffff);
                            
                            tft.setTextColor(TEXT_COLOR_1);
                            tft.setFontScale(TEXT_SIZE_SMALL);
                            if(newprogram1==1)
                            {
                                tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+11+39, 40);
                                tft.println("DMT Removal");
                                tft.setCursor(1*((DISPLAY_WIDTH/4)-10)+31+39, 40);
                                tft.println("Wash 2");
                                tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+28+39, 40);
                                tft.println("Elution");
                            }
                            if(newprogram2==1)
                            {
                                tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+26+39, 40);
                                tft.println("Elution");;
                            }
                            runScreenUpdate(run_scr);
                        }
                        
                        else
                        {
                            runScreen();
                            runScreenUpdate(run_scr);
                        }
                        
                        move(0,226,"NONE","FORWARD");
                        putchemin = 0;
                        delay(wait_time);
                        break;
                    }
                }     
            }
        }
    }  

    if(newprogram1==1)
    {
        tft.fillRect(2*((DISPLAY_WIDTH/4)-10)+2+39,35+29,(DISPLAY_WIDTH/4)-14,DISPLAY_HEIGHT-66-33,0xd6d6);
        tft.drawCircle(2*((DISPLAY_WIDTH/4)-10)+55+39,177,40,0x4545);
        tft.drawCircle(2*((DISPLAY_WIDTH/4)-10)+55+39,177,41,0x4545);
        tft.drawCircle(2*((DISPLAY_WIDTH/4)-10)+55+39,177,42,0x4545);
        tft.setFontScale(TEXT_SIZE_SMALL);
        tft.setTextColor(0x4545);
        tft.setCursor(2*((DISPLAY_WIDTH/4)-10)+20+39, 169);
        tft.println("COMPLETED");
    }
    if(newprogram2==1)
    {
        tft.fillRect(0*((DISPLAY_WIDTH/4)-10)+2+39,35+29,(DISPLAY_WIDTH/4)-14,DISPLAY_HEIGHT-66-33,0xd6d6);
        tft.drawCircle(0*((DISPLAY_WIDTH/4)-10)+55+39,177,40,0x4545);
        tft.drawCircle(0*((DISPLAY_WIDTH/4)-10)+55+39,177,41,0x4545);
        tft.drawCircle(0*((DISPLAY_WIDTH/4)-10)+55+39,177,42,0x4545);
        tft.setFontScale(TEXT_SIZE_SMALL);
        tft.setTextColor(0x4545);
        tft.setCursor(0*((DISPLAY_WIDTH/4)-10)+20+39, 169);
        tft.println("COMPLETED");
    }

    home("XY");
    tft.fillRoundRect(90,35,300,200,5,0xd6d6);
    tft.fillRoundRect(90,35,300,35,5,0x2525);
    tft.setTextColor(0xffff);
    tft.setFontScale(TEXT_SIZE_SMALL);
    tft.setCursor(201, 44);
    tft.println("COMPLETED");
    tft.setTextColor(0x0000);
    tft.setCursor(102, 103);
    tft.println("Press \"OK\" to back to main screen !");
    tft.fillRoundRect(200,189,75,25,5,0x2525);
    tft.setTextColor(0xffff);
    tft.setCursor(230, 193);
    tft.println("OK");
    completed = 1;
    while(completed==1)
    {
        if(tft.touchDetect())
        {  
            tft.touchReadPixel(&tx, &ty);
            if(tx>200 && tx<275 && ty>189 && ty<214)
            {
                resetFunc();
            }
        }
    }
    
}

void my_delay(int time_delay)
{
    count = 0;
    uint32_t old_time = count;
    while(count - old_time < time_delay)
    {
        //delay
        Serial.println(count-old_time);
    }    
}

void output(uint8_t chemical, uint8_t number, uint32_t volume)
{
    uint32_t pump_time = volume*400/260;
    if(chemical==1)
    { 
        if(number==1)
        { 
            digitalWrite(O1_PIN,HIGH);
            delay(pump_time);
            digitalWrite(O1_PIN,LOW);
            delay(wait_time);
        }
        if(number==2)
        { 
            digitalWrite(O1_PIN,HIGH);
            digitalWrite(O2_PIN,HIGH);
            delay(pump_time);
            digitalWrite(O1_PIN,LOW);
            digitalWrite(O2_PIN,LOW);
            delay(wait_time);
        }
        if(number==3)
        {
            digitalWrite(O1_PIN,HIGH);
            digitalWrite(O2_PIN,HIGH);
            digitalWrite(O3_PIN,HIGH);
            delay(pump_time);
            digitalWrite(O1_PIN,LOW);
            digitalWrite(O2_PIN,LOW);
            digitalWrite(O3_PIN,LOW);
            delay(wait_time);
        }
        if(number==4 || number==0)
        {
            digitalWrite(O1_PIN,HIGH);
            digitalWrite(O2_PIN,HIGH);
            digitalWrite(O3_PIN,HIGH);
            digitalWrite(O4_PIN,HIGH);
            delay(pump_time);
            digitalWrite(O1_PIN,LOW);
            digitalWrite(O2_PIN,LOW);
            digitalWrite(O3_PIN,LOW);
            digitalWrite(O4_PIN,LOW);
            delay(wait_time);
        }
    }

    if(chemical==2)
    { 
        if(number==1)
        { 
            digitalWrite(O5_PIN,HIGH);
            delay(pump_time);
            digitalWrite(O5_PIN,LOW);
            delay(wait_time);
        }
        if(number==2)
        { 
            digitalWrite(O5_PIN,HIGH);
            digitalWrite(O6_PIN,HIGH);
            delay(pump_time);
            digitalWrite(O5_PIN,LOW);
            digitalWrite(O6_PIN,LOW);
            delay(wait_time);
        }
        if(number==3)
        {
            digitalWrite(O5_PIN,HIGH);
            digitalWrite(O6_PIN,HIGH);
            digitalWrite(O7_PIN,HIGH);
            delay(pump_time);
            digitalWrite(O5_PIN,LOW);
            digitalWrite(O6_PIN,LOW);
            digitalWrite(O7_PIN,LOW);
            delay(wait_time);
        }
        if(number==4)
        {
            digitalWrite(O5_PIN,HIGH);
            digitalWrite(O6_PIN,HIGH);
            digitalWrite(O7_PIN,HIGH);
            digitalWrite(O8_PIN,HIGH);
            delay(pump_time);
            digitalWrite(O5_PIN,LOW);
            digitalWrite(O6_PIN,LOW);
            digitalWrite(O7_PIN,LOW);
            digitalWrite(O8_PIN,LOW);
            delay(wait_time);
        }
    }

    if(chemical==3)
    { 
        if(number==1)
        { 
            digitalWrite(O9_PIN,HIGH);
            delay(pump_time+30);
            digitalWrite(O9_PIN,LOW);
            delay(wait_time);
        }
        if(number==2)
        { 
            digitalWrite(O9_PIN,HIGH);
            digitalWrite(O10_PIN,HIGH);
            delay(pump_time+30);
            digitalWrite(O9_PIN,LOW);
            digitalWrite(O10_PIN,LOW);
            delay(wait_time);
        }
        if(number==3)
        {
            digitalWrite(O9_PIN,HIGH);
            digitalWrite(O10_PIN,HIGH);
            digitalWrite(O11_PIN,HIGH);
            delay(pump_time+30);
            digitalWrite(O9_PIN,LOW);
            digitalWrite(O10_PIN,LOW);
            digitalWrite(O11_PIN,LOW);
            delay(wait_time);
        }
        if(number==4)
        {
            digitalWrite(O9_PIN,HIGH);
            digitalWrite(O10_PIN,HIGH);
            digitalWrite(O11_PIN,HIGH);
            digitalWrite(O12_PIN,HIGH);
            delay(pump_time+30);
            digitalWrite(O9_PIN,LOW);
            digitalWrite(O10_PIN,LOW);
            digitalWrite(O11_PIN,LOW);
            digitalWrite(O12_PIN,LOW);
            delay(wait_time);
        }
    }
}

void move(float x_step, float y_step, char *x_direction, char *y_direction)
{
    uint32_t x_pulse, y_pulse;
    x_pulse = (uint32_t)(x_step*3200/4);
    y_pulse = (uint32_t)(y_step*3200/4);

    digitalWrite(X_ENABLE_PIN, LOW);
    digitalWrite(Y_ENABLE_PIN, LOW);

    if(x_direction=="RIGHT")
        digitalWrite(X_DIR_PIN, HIGH);
    if(x_direction=="LEFT")
        digitalWrite(X_DIR_PIN, LOW);
    if(y_direction=="FORWARD")
        digitalWrite(Y_DIR_PIN, HIGH);
    if(y_direction=="REVERSE")
        digitalWrite(Y_DIR_PIN, LOW);
    
    if(x_pulse > y_pulse)
    { 
        for(uint32_t m=0; m<x_pulse; m++)
        { 
            if(m<y_pulse)
            {
                digitalWrite(X_STEP_PIN,HIGH);
                digitalWrite(Y_STEP_PIN,HIGH);
                delayMicroseconds(10);
                digitalWrite(X_STEP_PIN,LOW);
                digitalWrite(Y_STEP_PIN,LOW);
                delayMicroseconds(40);
            }
            else
            {
                digitalWrite(X_STEP_PIN,HIGH);
                delayMicroseconds(10);
                digitalWrite(X_STEP_PIN,LOW);
                delayMicroseconds(40);
            }
        }  
    }

    if(x_pulse < y_pulse)
    { 
        for(uint32_t m=0; m<y_pulse; m++)
        { 
            if(m<x_pulse)
            {
                digitalWrite(X_STEP_PIN,HIGH);
                digitalWrite(Y_STEP_PIN,HIGH);
                delayMicroseconds(10);
                digitalWrite(X_STEP_PIN,LOW);
                digitalWrite(Y_STEP_PIN,LOW);
                delayMicroseconds(40);
            }
            else
            {
                digitalWrite(Y_STEP_PIN,HIGH);
                delayMicroseconds(10);
                digitalWrite(Y_STEP_PIN,LOW);
                delayMicroseconds(40);
            }
        }  
    }

    if(x_pulse == y_pulse)
    { 
        for(uint32_t m=0; m<x_pulse; m++)
        { 
            digitalWrite(X_STEP_PIN,HIGH);
            digitalWrite(Y_STEP_PIN,HIGH);
            delayMicroseconds(10);
            digitalWrite(X_STEP_PIN,LOW);
            digitalWrite(Y_STEP_PIN,LOW);
            delayMicroseconds(40);
        }  
    }

    digitalWrite(X_ENABLE_PIN, HIGH);
    digitalWrite(Y_ENABLE_PIN, HIGH);
}
//////////////////// STEPPER - END ////////////////////

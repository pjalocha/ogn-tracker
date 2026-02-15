#include "main.h"

#include "play.h"
#include <fifo.h>

#ifdef WITH_BEEPER

// #define WITH_BEEPER_GEN

#ifdef WITH_BEEPER_GEN   // if buzzer with internal generator is used

void Beep_Init(void)
{ pinMode(Buzzer_Pin, OUTPUT);
  digitalWrite(Buzzer_Pin, LOW); }

void Beep(uint16_t Freq, uint8_t Duty, uint8_t DoubleAmpl)
{ digitalWrite(Buzzer_Pin, Freq>0); }

#else

void Beep_Init(void)
{ ledcSetup(Buzzer_Channel, 800, 8);      // channel, frequency, resolution
  pinMode(Buzzer_Pin, OUTPUT);
  digitalWrite(Buzzer_Pin, LOW); }

void Beep(uint16_t Freq, uint8_t Duty, uint8_t DoubleAmpl) // [Hz, 1/256] play sound with given frequency and duty (=volume)
{ if(Freq==0)
  { ledcDetachPin(Buzzer_Pin);
    pinMode(Buzzer_Pin, OUTPUT);
    digitalWrite(Buzzer_Pin, LOW); }
  else
  { ledcAttachPin(Buzzer_Pin, Buzzer_Channel); }
    ledcWriteTone(Buzzer_Channel, Freq);
    ledcWrite(Buzzer_Channel, Duty);
}

#endif

// Frequencies for notes of the highest octave: C,     C#,    D,     D#,    E,     F,     F#,    G,     G#,    A,     A#,    B
// Freq[i] = 32*523.25*2**(i/12)            i = 0,     1,     2,     3,     4,     5,     6,     7,     8,     9,     A,     B
static const uint16_t NoteFreq[12] =      { 16744, 17740, 18795, 19912, 21096, 22351, 23680, 25088, 26579, 28160, 29834, 31608 } ;

void Beep_Note(uint8_t Note) // Note = VVOONNNN: VV = Volume, OO=Octave, NNNN=Note
{ uint8_t Volume =  Note>>6;                             // [0..3]           // 2 volume bits
  uint8_t Octave = (Note>>4)&0x03;                       // [0..3]           // 2 octave bits
  Note &= 0x0F; if(Note>=12) { Note-=12; Octave+=1; }    // [0..11] [0..4]   // 4 note bits
  uint8_t Duty = 0; uint8_t DoubleAmpl=0;
  if(Volume) { Duty=0x10; Duty<<=Volume; }               // Volume => Duty = 0x00, 0x20, 0x40, 0x80
  if(Volume>2) { DoubleAmpl=1; }                         // DoubleAmpl = 0, 0, 1, 1
  uint16_t Freq = NoteFreq[Note];
  // if(Octave) { /* Freq += 1<<(Octave-1); */ Freq >>= (5-Octave); }
  Freq >>= (5-Octave);
  if(Duty==0) Freq=0;
  Beep(Freq, Duty, DoubleAmpl); }

// static uint8_t  Vario_Note=0x00; // 0x40;
// static uint16_t Vario_Period=800;
// static uint16_t Vario_Fill=50;

// static volatile uint16_t Vario_Time=0;

static volatile uint8_t Play_Note=0;             // Note being played
static volatile uint8_t Play_Counter=0;          // [ms] time counter

static FIFO<uint16_t, 16> Play_FIFO;             // queue of notes to play

void Play(uint8_t Note, uint8_t Len)             // [Note] [ms] put a new note to play in the queue
{ uint16_t Word = Note; Word<<=8; Word|=Len; Play_FIFO.Write(Word); }

uint8_t Play_isBusy(void) { return Play_Counter; } // is a note being played right now ?

void Play_TimerCheck(uint8_t Ticks)              // every ms serve the note playing
{ uint8_t Counter=Play_Counter;
  if(Counter)                                    // if counter non-zero
  { if(Counter>Ticks) Counter-=Ticks;            // decrement it
                 else Counter=0;
    if(Counter==0) Beep_Note(Play_Note=0x00);    // if reached zero, stop playing the note
  }
  if(Counter==0)                                 // if counter reached zero
  { if(!Play_FIFO.isEmpty())                     // check for notes in the queue
    { uint16_t Word=0; Play_FIFO.Read(Word);     // get the next note
      Beep_Note(Play_Note=Word>>8); Counter=Word&0xFF; }   // start playing it, load counter with the note duration
  }
  Play_Counter=Counter;

  // uint16_t Time=Vario_Time;
  // Time++; if(Time>=Vario_Period) Time=0;
  // Vario_Time = Time;

  if(Counter==0)                            // when no notes are being played, make the vario sound
  { // if(Time<=Vario_Fill)
    // { if(Play_Note!=Vario_Note) Beep_Note(Play_Note=Vario_Note); }
    // else
    { if(Play_Note!=0) Beep_Note(Play_Note=0x00); }
  }
}

#endif


#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>

// ===================================================================================================

class DDB_ID
{ public:
   uint32_t ID;         // aircraft ID (24-bit address + address-type in the top byte)
   char AcftModel[16];  // aircraft model
   char TailCall [ 8];  // tail/competition call-sign
   char RegCall  [13];  // aircraft registration call-sign
   union
   { uint8_t Flags;
     struct
     { bool DoTrack:1;
       bool DoIdent:1;
     } ;
   } ;

   DDB_ID()
   { ID=0; RegCall[0]=0; TailCall[0]=0; AcftModel[0]=0; Flags=0; }

   void setAcftModel(const char *Model, int Len) { if(Len>15) Len=15; memcpy(AcftModel, Model, Len); AcftModel[Len]=0; }
   void setRegCall  (const char *Call , int Len) { if(Len>12) Len=12; memcpy(RegCall,   Call,  Len); RegCall  [Len]=0; }
   void setTailCall (const char *Call , int Len) { if(Len> 7) Len= 7; memcpy(TailCall,  Call,  Len); TailCall [Len]=0; }

   uint32_t getAddress (void) const { return ID&0xFFFFFF; }
   uint8_t  getAddrType(void) const { return (ID>>24)&0xFF; }

   int Read(char *Line)
   { int LineLen=strlen(Line);
     if(LineLen<8) return 0;
     if(Line[0]=='#') return 0;
     char *Token = strtok(Line, ","); // device-type as a letter: I,F,O
     if(Token==0) return 0;
     char AddrType;
     if(sscanf(Token, "'%c'", &AddrType)!=1) return 0;
     Token = strtok(0, ",");          // device address: hex
     if(Token==0) return 0;
     unsigned Addr;
     if(sscanf(Token, "'%06x'", &Addr)!=1) return 0;
     ID = Addr&0xFFFFFF;
     switch(AddrType)
     { case 'I': ID|=0x05000000; break;
       case 'F': ID|=0x06000000; break;
       case 'O': ID|=0x07000000; break;
       default: return 0; }
     Token = strtok(0, ",");          // aircraft model
     if(Token==0 || Token[0]!='\'') return 0;
     int Len=strlen(Token);
     if(Len<2 || Token[Len-1]!='\'') return 0;
     setAcftModel(Token+1, Len-2);
     Token = strtok(0, ",");          // registration call
     if(Token==0 || Token[0]!='\'') return 0;
     Len=strlen(Token);
     if(Len<2 || Token[Len-1]!='\'') return 0;
     setRegCall(Token+1, Len-2);
     Token = strtok(0, ",");          // competition call
     if(Token==0 || Token[0]!='\'') return 0;
     Len=strlen(Token);
     if(Len<2 || Token[Len-1]!='\'') return 0;
     setTailCall(Token+1, Len-2);
     Token = strtok(0, ",");          // track or not
     if(Token==0 || Token[0]!='\'' || Token[2]!='\'') return 0;
     DoTrack = Token[1]=='Y';
     Token = strtok(0, ",\n\r");      // identify or not
     if(Token==0 || Token[0]!='\'' || Token[2]!='\'') return 0;
     DoIdent = Token[1]=='Y';
     return 1; }
} ;

// ===================================================================================================

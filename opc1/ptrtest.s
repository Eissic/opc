          ORG 0x10
RESPTR    BYTE 0
          ORG 0x20
RESLOC    BYTE 0

          ORG 0x100
          lda.i RESLOC
          sta   RESPTR

          lda.i 0xF0
          sta.p RESPTR
        


          halt

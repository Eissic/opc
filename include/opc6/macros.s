# -----------------------------------------------------------------------------
# Macros
# -----------------------------------------------------------------------------

# set the cpu type to support conditional assembly
##define CPU_OPC6

EQU     EI_MASK, 0x0008
EQU    SWI_MASK, 0x00F0
EQU   SWI0_MASK, 0x0010
EQU   SWI1_MASK, 0x0020
EQU   SWI2_MASK, 0x0040
EQU   SWI3_MASK, 0x0080
        
MACRO   CPU_STRING()
    STRING "OPC6"
ENDMACRO

MACRO   CPU_BSTRING()
    BSTRING "OPC6"
ENDMACRO

MACRO   CLC()
    c.add r0,r0
ENDMACRO

MACRO   SEC()
    nc.ror r0,r0,1
ENDMACRO

MACRO   GETPSR(_reg_)
    getpsr  _reg_, psr
ENDMACRO

MACRO   PUTPSR(_reg_)
    putpsr  psr, _reg_
ENDMACRO

MACRO   SWI0() {
    putpsr  psr, r0, SWI0_MASK
ENDMACRO
        
MACRO   SWI1() {
    putpsr  psr, r0, SWI1_MASK
ENDMACRO
        
MACRO   SWI2() {
    putpsr  psr, r0, SWI2_MASK
ENDMACRO
        
MACRO   SWI3() {
    putpsr  psr, r0, SWI3_MASK
ENDMACRO
        
MACRO   EI()
    GETPSR  (r12)
    or      r12, r12, EI_MASK
    PUTPSR  (r12)
ENDMACRO

MACRO   DI()
    GETPSR  (r12)
    and     r12, r12, ~EI_MASK
    PUTPSR  (r12)
ENDMACRO

MACRO JSR( _address_)
    jsr     r13, r0, _address_
ENDMACRO

MACRO RTS()
    mov     pc, r13
ENDMACRO

MACRO   PUSH( _data_)
    mov     r14, r14, -1
    sto     _data_, r14, 1
ENDMACRO

MACRO   POP( _data_ )
    ld      _data_, r14, 1
    mov     r14, r14, 1
ENDMACRO

MACRO   IN(_reg_, _address_)
    in      _reg_, r0, _address_
ENDMACRO
        
MACRO   OUT(_reg_, _address_)
    out     _reg_, r0, _address_
ENDMACRO
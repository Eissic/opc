module opc6cpu(
    input[15:0] din,            // 数据输入信号
    input clk,                  // 时钟信号
    input reset_b,              // 复位信号
    input[1:0] int_b,           // 中断
    input clken,                // 时钟使能
    output vpa,
    output vda,
    output vio,
    output[15:0] dout,          // 数据输出信号
    output[15:0] address,       // 地址信号
    output rnw);                // 写或读信号
    
    parameter MOV=5'h0,AND=5'h1,OR=5'h2,XOR=5'h3,ADD=5'h4,ADC=5'h5,STO=5'h6,LD=5'h7,ROR=5'h8,JSR=5'h9,SUB=5'hA,SBC=5'hB,INC=5'hC,LSR=5'hD,DEC=5'hE,ASR=5'hF;
    parameter HLT=5'h10,BSWP=5'h11,PPSR=5'h12,GPSR=5'h13,RTI=5'h14,NOT=5'h15,OUT=5'h16,IN=5'h17,PUSH=5'h18,POP=5'h19,CMP=5'h1A,CMPC=5'h1B;    
                                // PPSR is "Put Processor Status Register", GPSR is "Get ..."

    parameter FET0=3'h0,FET1=3'h1,EAD=3'h2,RDM=3'h3,EXEC=3'h4,WRM=3'h5,INT=3'h6;    // 定义FDM状态
    parameter EI=3,S=2,C=1,Z=0,IRLEN=12,IRLD=16,IRSTO=17,IRNPRED=18,IRWBK=19,INT_VECTOR0=16'h0002,INT_VECTOR1=16'h0004;
    parameter P0=15,P1=14,P2=13;    // predict用的三位bit

    reg [15:0] OR_q;            // 
    reg [15:0] PC_q;            // program counter
    reg [15:0] PCI_q;           // 
    reg [15:0] result;          // ALU运算结果
    reg [19:0] IR_q;            // 
    
    //(* RAM_STYLE="DISTRIBUTED" *)
    reg [15:0] RF_q[15:0];      // 16 general purpose registers
    reg [2:0]  FSM_q;           // 有限状态机状态
    reg [3:0]  swiid;           // 
    reg [3:0]  PSRI_q;          // 
    reg [7:0]  PSR_q ;          // 处理器状态寄存器
    reg        zero,carry,sign,enable_int;      // CPU当前状态
    reg        reset_s0_b,reset_s1_b,pred_q;    // 
    
    wire [4:0]  op            = {IR_q[IRNPRED],IR_q[11:8]};
    wire [4:0]  op_d          = { (din[15:13]==3'b001),din[11:8] };
    wire        pred_d        = (din[15:13]==3'b001) || (din[P2] ^ (din[P1] ? (din[P0] ? sign : zero): (din[P0] ? carry : 1))); // New data,new flags (in exec/fetch)
    wire        pred_din      = (din[15:13]==3'b001) || (din[P2] ^ (din[P1] ? (din[P0] ? PSR_q[S]:PSR_q[Z]):(din[P0] ? PSR_q[C]:1))); // New data,old flags (in fetch0)
    wire [15:0] RF_w_p2       = (IR_q[7:4]==4'hF) ? PC_q: {16{(IR_q[7:4]!=4'h0)}} & RF_q[IR_q[7:4]];                          // Port 2 always reads source reg
    wire [15:0] RF_dout       = (IR_q[3:0]==4'hF) ? PC_q: {16{(IR_q[3:0]!=4'h0)}} & RF_q[IR_q[3:0]];                          // Port 1 always reads dest reg
    wire [15:0] operand       = (IR_q[IRLEN]||IR_q[IRLD]||(op==INC)||(op==DEC)||(IR_q[IRWBK]))?OR_q:RF_w_p2;     // One word instructions operand usu comes from RF
    
    // assign {rnw,dout,address} = {!(FSM_q==WRM), RF_w_p2,(FSM_q==WRM||FSM_q==RDM)? ((op==POP)? RF_dout: OR_q)  : PC_q};
    assign rnw                = !(FSM_q==WRM);
    assign dout               = RF_w_p2;
    assign address            = (FSM_q==WRM||FSM_q==RDM)? ((op==POP)? RF_dout: OR_q)  : PC_q;

    // assign {vpa,vda,vio}      = {((FSM_q==FET0)||(FSM_q==FET1)||(FSM_q==EXEC)),({2{(FSM_q==RDM)||(FSM_q==WRM)}} & {!((op==IN)||(op==OUT)),(op==IN)||(op==OUT)})};
    assign vpa                = (FSM_q==FET0)||(FSM_q==FET1)||(FSM_q==EXEC);            // program address
    assign vda                = ((FSM_q==RDM)||(FSM_q==WRM)) & !((op==IN)||(op==OUT));  // data address
    assign vio                = ((FSM_q==RDM)||(FSM_q==WRM)) & ((op==IN)||(op==OUT));   // io address
    
    // ALU calcu
    always @( * ) begin
        case (op)
            AND,OR               :{carry,result} = {PSR_q[C],(IR_q[8])?(RF_dout & operand):(RF_dout | operand)};
            ADD,ADC,INC          :{carry,result} = RF_dout + operand + (IR_q[8] & PSR_q[C]);
            SUB,SBC,CMP,CMPC,DEC :{carry,result} = RF_dout + (operand ^ 16'hFFFF) + ((IR_q[8])?PSR_q[C]:1);
            XOR,GPSR             :{carry,result} = (IR_q[IRNPRED])?{PSR_q[C],8'b0,PSR_q}:{PSR_q[C],RF_dout ^ operand};
            NOT,BSWP             :{result,carry} = (IR_q[10])? {~operand,PSR_q[C]} : {operand[7:0],operand[15:8],PSR_q[C]};
            ROR,ASR,LSR          :{result,carry} = {(IR_q[10]==0)?PSR_q[C]:(IR_q[8]==1)?operand[15]:1'b0,operand};
            default              :{carry,result} = {PSR_q[C],operand} ; //LD,MOV,STO,JSR,IN,OUT,PUSH,POP and everything else
        endcase // case ( IR_q )
        {swiid,enable_int,sign,carry,zero} = (op==PPSR)?operand[7:0]:(IR_q[3:0]!=4'hF)?{PSR_q[7:3],result[15],carry,!(|result)}:PSR_q;
    end // always @ ( * )
    
    // finite state machine
    always @(posedge clk)
        if (clken) begin
            {reset_s0_b,reset_s1_b,pred_q} <= {reset_b,reset_s0_b,(FSM_q==FET0)?pred_din:pred_d};
            if (!reset_s1_b)
                {PC_q,PCI_q,PSRI_q,PSR_q,FSM_q} <= 0;
            else begin
                case (FSM_q)
                    FET0   : FSM_q <= (din[IRLEN]) ? FET1 : (!pred_din) ? FET0 : ((din[11:8]==LD)||(din[11:8]==STO)||(op_d==PUSH)||(op_d==POP)) ? EAD : EXEC;
                    // Is it a long instruction?    YES: change to FET1 state           NO: judge next line ↓
                    // Is prediction true?          YES: judge next line ↓              NO: change to FET0 state to refetch
                    // Is it related to MEM op?     YES: change to EDA state            NO: change to EXEC state
                    
                    FET1   : FSM_q <= (!pred_q )? FET0: ((IR_q[3:0]!=0) || (IR_q[IRLD]) || IR_q[IRSTO])?EAD:EXEC;
                    // Is prediction true?          YES: judge next line ↓              NO: change to FET0 state to refetch
                    // Is IRLD bit or ITSTO bit or dst Resgister isn't R0?
                    //                              YES: change to EAD state            NO: change to EXEC statep
                    
                    EAD    : FSM_q <= (IR_q[IRLD]) ? RDM : (IR_q[IRSTO]) ? WRM : EXEC;
                    // Is IRLD bit is H?         YES: change to RDM state            NO: judge next line ↓ 
                    // Is IRSTO bit is H?        YES: change to WRM state            NO: change to EXEC state

                    EXEC   : FSM_q <= ((!(&int_b) & PSR_q[EI])||((op==PPSR) && (|swiid)))?INT:((IR_q[3:0]==4'hF)||(op==JSR))?FET0:
                                      (din[IRLEN]) ? FET1 : ((din[11:8]==LD)||(din[11:8]==STO)||(op_d==POP)||(op_d==PUSH))?EAD:(pred_d)?EXEC:FET0;
                    // Is it a hardware or software intrupt?            YES: change to INT state        NO: judge next line ↓ 
                    // Is it a JSR op?                                  YES: change to FET0 state       NO: judge next line ↓ 
                    // Is it a long op?                                 YES: change to FET1 state       NO: judge next line ↓ 
                    // Is it a mem op(LD, STO, PUSH, POP)?              YES: change to EAD state        NO: judge next line ↓
                    // Is prediction true?                              YES: stay in EXEC state         NO: change to FET0
                    //****TODO: have some question about these line: why need rejudge the prediction and why it may stay at EXEC state?

                    WRM    : FSM_q <= (!(&int_b) & PSR_q[EI])?INT:FET0;
                    default: FSM_q <= (FSM_q==RDM)? EXEC : FET0;  // Applies to INT and RDM plus undefined states
                endcase 
                
                OR_q <= ((FSM_q==FET0)||(FSM_q==EXEC))?({16{op_d==PUSH}}^({12'b0,(op_d==DEC)||(op_d==INC)?din[7:4]:{3'b0,(op_d==POP)}})):(FSM_q==EAD)?RF_w_p2+OR_q:din;
                // 
                
                if ( FSM_q == INT )
                    {PC_q,PCI_q,PSRI_q,PSR_q[EI]} <= {(!int_b[1])?INT_VECTOR1:INT_VECTOR0,PC_q,PSR_q[3:0],1'b0} ; 
                    // Always clear EI on taking interrupt
                
                else if ((FSM_q==FET0)||(FSM_q==FET1)) 
                    PC_q  <= PC_q + 1;

                else if ( FSM_q == EXEC) begin
                    PC_q <= (op==RTI)?PCI_q: ((IR_q[3:0]==4'hF)||(op==JSR))?result:(((!(&int_b)) && PSR_q[EI])||((op==PPSR)&&(|swiid)))?PC_q:PC_q + 1;
                    // Is op RTI(return from interupt)?                     YES: PC_q <= PCI_q                  NO: judge next line ↓ 
                    // Is oprating PC register?                             YES: PC_q <= result(ALU result)     NO: judge next line ↓ 
                    // Is one of int_b(inpuft) bits is L and EI bit is H？
                    // Or is op Put PSR and one of swiid bits H?
                    // In general: Is it a hardware or software intrupt?    YES: PC_q <= PC_q                   NO: PC_q++       
                    
                    PSR_q <= (op==RTI)?{4'b0,PSRI_q}:{swiid,enable_int,sign,carry,zero}; // Clear SWI bits on return
                end
                
                if (((FSM_q==EXEC) && !((op==CMP)||(op==CMPC))) || (((FSM_q==WRM)||(FSM_q==RDM)) && IR_q[IRWBK]))
                    RF_q[IR_q[3:0]] <= (op==JSR)? PC_q : result ;
                
                if ((FSM_q==FET0)||(FSM_q==EXEC))
                    IR_q <= {(op_d==PUSH)||(op_d==POP),(din[15:13]==3'b001),(din[11:8]==STO)||(op_d==PUSH),(din[11:8]==LD)||(op_d==POP),din};
                    // IR_q is 20 bits long.
                    // 0~15: the first (or only) word of instruction
                    // 16: load operation flag
                    // 17: store op flag
                    // 18: predication flag
                    // 19: writeback flag
                else if ((FSM_q==EAD && (IR_q[IRLD]||IR_q[IRSTO]))||(FSM_q==RDM))
                  IR_q[7:0] <= {IR_q[3:0],IR_q[7:4]}; // Swap source/dest reg in EA for reads and writes for writeback of 'source' in push/pop .. swap back again in RDMEM
            end 
        end
endmodule

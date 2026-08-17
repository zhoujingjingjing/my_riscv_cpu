`include "mycpu_top.h"

/*
文件职责：ID 级把机器码翻译成后端需要的控制信号，并决定两条指令能不能配对发射。
上游输入来自 IF2，输出送给 ISSUE 和 IF；CSR 冲突记录也在本级处理。
上游是 IF2 的顺序取指队列，下游是 ISSUE；寄存器值和数据前递不在这里处理。
*/
module id_stage(
    input wire clk,
    input wire reset,

    /*ID 只负责把指令翻译成控制信号。
    rs1/rs2 的地址走窄总线给 ISSUE，真正的寄存器值在那里读取和前递。*/
    output wire [`ID_TO_ISSUE_BUS_WIDTH-1:0] id_to_issue_bus,
    output wire [`ID_TO_ISSUE_BUS_WIDTH-1:0] id_to_issue_bus1,
    output wire [`ID_TO_ISSUE_RF_BUS_WIDTH-1:0] id_to_issue_rf_bus,
    output wire [`ID_TO_IF_BUS_WIDTH-1:0] id_to_if_bus,
    input  wire [`IF_TO_ID_BUS_WIDTH-1:0]  if_to_id_bus,
    /*if_to_id_bus1 是 FIFO 里的第二条候选指令。
    只有它和 slot0 配对成功，才会一起进入后端流水线。*/
    input  wire [`FETCH_BUS_WIDTH-1:0] if_to_id_bus1,
    /*IF2 随两条指令送来的轻量预译码标签。
    它们在 IROM 返回时就已经算好，并在 FIFO 中和对应机器码一起移动。*/
    input  wire [24:0] if_to_id_dec0,
    input  wire [24:0] if_to_id_dec1,
    /*wb_ex/mret_flush 来自 WB，用来清空 ID 中已经走错的指令。
    flush_en 来自 EXE，表示分支预测错了。三者都不会让旧指令继续向前。*/
    input  wire        wb_ex,
    input  wire        mret_flush,
    input  wire        flush_en,

    output wire id_allow_in,
    output wire id_take2,
    output wire id_to_issue_valid,
    output wire id_to_issue_valid1,
    input wire issue_allow_in,
    input wire if_to_id_valid,
    input wire if_to_id_valid1,
    /*perf_pair_* 只是统计信号，不参与流水线控制。
    它们只在 slot0 真正交给 ISSUE 的那一拍产生一次，ID 停住时不会重复计数。*/
    output wire perf_pair,
    output wire perf_pair_ok,
    output wire perf_pair_raw,
    output wire perf_pair_waw,
    output wire perf_pair_lsu,
    output wire perf_pair_mul,
    output wire perf_pair_ctrl,
    output wire perf_pair_serial

);



    wire wb_flush = wb_ex || mret_flush;

    /*流水控制：id_valid 表示 ID 里有一条有效指令，shadow_valid 表示旁边还有第二条。
    ID 能不能接收新指令由 id_allow_in 决定；能不能把当前指令交给 ISSUE 由 id_ready_go 决定。*/
    wire id_ready_go;
    reg  id_valid;
    reg [`IF_TO_ID_BUS_WIDTH-1:0] id_reg;
    reg [24:0] id_dec_reg;
    
    /*shadow_reg 是 slot1 的临时座位。
    slot0 和 slot1 在同一个时钟沿锁存，保证它们属于同一组指令。*/
    reg [`FETCH_BUS_WIDTH-1:0] shadow_reg;
    reg [24:0] shadow_dec_reg;
    reg shadow_valid;
    always @ (posedge clk) begin
        if (reset) begin
            id_valid <= 1'b0;
        end else if (wb_flush || flush_en) begin
            /*异常、mret 或分支纠错时，当前 ID 指令作废，变成一个气泡。*/
            id_valid <= 1'b0;
        end else if (id_allow_in) begin
            id_valid <= if_to_id_valid;
        end
    end

    // ID没有需要等待的普通数据；CSR串行冲突由下面的csr_hazard控制。
    assign id_allow_in = ~id_valid || id_ready_go && issue_allow_in;
    assign id_to_issue_valid = id_valid && id_ready_go && !flush_en && !wb_flush;
    /*slot1 只有在 slot0 有效、shadow_valid 有效并且 pair_ok 为 1 时才算真指令。*/
    assign id_to_issue_valid1 = id_to_issue_valid && shadow_valid && pair_ok;


    /*接收 IF2：只有 ID 能接收时才锁存新的 slot0/slot1。
    如果 ID 被堵住，输入总线保持原样，避免把队头指令冲掉。*/
    always @(posedge clk) begin
        if (if_to_id_valid && id_allow_in) begin
            id_reg <= if_to_id_bus;
            id_dec_reg <= if_to_id_dec0;
        end
    end

    always @(posedge clk) begin
        if (reset || wb_flush || flush_en) begin
            shadow_valid <= 1'b0;
        end else if (id_allow_in) begin
            shadow_valid <= if_to_id_valid && if_to_id_valid1;
            if (if_to_id_valid && if_to_id_valid1) begin
                shadow_reg <= if_to_id_bus1;
                shadow_dec_reg <= if_to_id_dec1;
            end
        end
    end

    /*slot0 的基本资料：
    id_pc/id_inst 是地址和机器码；pre_* 是 IF 当时的分支预测；
    current_ras_ptr 是取指时的 RAS 位置，预测错时用来恢复。*/
    wire [31:0] id_pc;
    wire [31:0] id_inst;

    wire        pre_taken;
    wire [31:0] pre_target;
    wire [5:0]  pre_index;
    wire [2:0]  current_ras_ptr;// 用于拆出 IF 传过来的快照
    assign {id_pc, id_inst, pre_taken, pre_target, pre_index, current_ras_ptr} = id_reg;//输入

    /*shadow_* 是 slot1 自己的地址、机器码和预测信息。
    pair_ok 为 0 时，slot1 不会丢掉；它会留在 FIFO 队头，下一拍变成新的 slot0。*/
    wire [31:0] shadow_pc;
    wire [31:0] shadow_inst;
    wire        shadow_taken;
    wire [31:0] shadow_target;
    wire [5:0]  shadow_index;
    assign {shadow_pc, shadow_inst, shadow_taken,
            shadow_target, shadow_index} = shadow_reg;

    //============================================================
    // dual-slot predecode and pairing
    //============================================================


    /*ID 需要在当前时钟沿决定 FIFO 弹一条还是两条，所以直接看 IF2 队头
    带来的两张预译码标签。这里不再从机器码重新跑 decoder，缩短 FIFO
    到 id_take2 再返回 FIFO 的组合路径。*/
    wire in_pair_ok;
    wire in_late;
    wire in_raw, in_waw, in_lsu, in_mul, in_ctrl, in_serial;
    dual_issue_ctrl u_in_pair (
        .valid0(if_to_id_valid), .valid1(if_to_id_valid1),
        .dec0(if_to_id_dec0), .dec1(if_to_id_dec1), .pair_ok(in_pair_ok),
        .late(in_late),
        .block_raw(in_raw), .block_waw(in_waw), .block_lsu(in_lsu),
        .block_mul(in_mul), .block_ctrl(in_ctrl), .block_serial(in_serial)
    );
    assign id_take2 = if_to_id_valid && if_to_id_valid1 && in_pair_ok;


    /*当前 ID 组也直接使用随指令锁存的标签。这样 slot0/slot1 即使被
    ISSUE 反压留在 ID，配对结论仍和进入 FIFO 时的原始机器码一一对应。*/
    wire [24:0] pair_dec0 = id_dec_reg;
    wire [24:0] pair_dec1 = shadow_dec_reg;
    wire pair_ok;
    wire pair_late;
    wire pair_block_raw;
    wire pair_block_waw;
    wire pair_block_lsu;
    wire pair_block_mul;
    wire pair_block_ctrl;
    wire pair_block_serial;

    dual_issue_ctrl u_pair_ctrl (
        .valid0(id_valid),
        .valid1(shadow_valid),
        .dec0(pair_dec0),
        .dec1(pair_dec1),
        .pair_ok(pair_ok),
        .late(pair_late),
        .block_raw(pair_block_raw),
        .block_waw(pair_block_waw),
        .block_lsu(pair_block_lsu),
        .block_mul(pair_block_mul),
        .block_ctrl(pair_block_ctrl),
        .block_serial(pair_block_serial)
    );


    /*ISSUE 需要四个源寄存器地址和四个“是否真的要读”的标志。
    这些信息单独走窄总线；slot0/slot1 的配对标签都来自 IF2 随指令保存的结果。*/
    wire use_rf_rdata1;
    wire use_rf_rdata2;
    wire [11:0] lane1_rf;
    wire [`ID_DATA_BUS_WIDTH-1:0] lane1_bus;
    wire lane1_push;
    wire lane1_pop;
    id_lane u_lane1 (
        .valid(id_to_issue_valid1),
        .pc(shadow_pc), .inst(shadow_inst),
        .pre_taken(shadow_taken), .pre_target(shadow_target),
        .pre_index(shadow_index), .ras_ptr(current_ras_ptr),
        .bus(lane1_bus), .rf_bus(lane1_rf),
        .push_ras(lane1_push), .pop_ras(lane1_pop)
    );

    assign id_to_issue_rf_bus = {
        use_rf_rdata1, use_rf_rdata2, rs1, rs2,
        lane1_rf
    };


    /*pair_late 表示一种可以直接处理的同组 RAW：
    slot0 是普通 ALU，slot1 读取 slot0 的 rd。ISSUE/MEM 会把 slot0 的结果转给 slot1，
    所以这里只记录 slot1 的 rs1、rs2 哪一个需要这份结果。*/
    wire pair_dep1 = pair_late && pair_dec1[23] &&
                     (pair_dec1[14:10] == pair_dec0[4:0]);
    wire pair_dep2 = pair_late && pair_dec1[22] &&
                     (pair_dec1[9:5] == pair_dec0[4:0]);
    assign id_to_issue_bus1 = {pair_late, pair_dep1, pair_dep2, lane1_bus};

    /*id_to_issue_valid 表示 ID 已经准备好；issue_allow_in 表示 ISSUE 现在能接收。
    两者同时为 1 才算真正发射，pair_event 用这个时刻统一产生配对统计。*/
    wire pair_event = id_to_issue_valid && issue_allow_in && shadow_valid;
    assign perf_pair        = pair_event;
    assign perf_pair_ok     = pair_event && pair_ok;
    assign perf_pair_raw    = pair_event && pair_block_raw;
    assign perf_pair_waw    = pair_event && pair_block_waw;
    assign perf_pair_lsu    = pair_event && pair_block_lsu;
    assign perf_pair_mul    = pair_event && pair_block_mul;
    assign perf_pair_ctrl   = pair_event && pair_block_ctrl;
    assign perf_pair_serial = pair_event && pair_block_serial;

    //============================================================
    // slot0 control signal declarations
    //============================================================

    /*slot0 的完整控制包：先在 ID 译码，后面每一级只按固定位置取用。
    这里的 rs1_value/rs2_value 先填 0，真正的寄存器值由 ISSUE 读取和前递。*/


    /*alu_op 选择 ALU 做哪一种运算。
    src1_is_pc/src2_is_imm 选择 ALU 两个输入来自 PC、寄存器还是立即数。
    res_from_mem 表示写回值来自 load 数据；mem_en/mem_we 描述访存；
    reg_we/reg_waddr 描述最后是否写 rd 以及写哪个寄存器。
    这些信号现在只是“工作单”，要等指令走到 EXE/MEM/WB 才真正生效。*/
    wire [11:0] alu_op;
    wire        src1_is_pc;
    wire        src2_is_imm;
    wire        res_from_mem;
    wire        mem_en;
    wire [3:0]  mem_we;
    wire        reg_we;
    wire [4:0]  reg_waddr;
    /*ID 不读寄存器值，所以先把总线中的两个操作数位置填 0。
    ISSUE 会在同一条总线的这两个位置换入读出的值。*/
    wire [31:0] rs1_value = 32'b0;
    wire [31:0] rs2_value = 32'b0;
    wire [31:0] imm;//ALU的src2的立即数值

    /*这八个小旗子说明访存宽度：
    lb/lbu 是字节，lh/lhu 是半字，lw 是整字；sb/sh/sw 是写字节、半字、整字。
    MEM 根据它们选择哪几个字节，并决定 load 要不要符号扩展。*/
    wire        inst_lb;
    wire        inst_lh;
    wire        inst_lw;
    wire        inst_lbu;
    wire        inst_lhu;
    wire        inst_sb;
    wire        inst_sh;
    wire        inst_sw;
    /*这些信号告诉 EXE 当前是哪一种分支或跳转。
    EXE 用它们比较条件、计算目标地址，再把真实结果送回 BTB。*/
    wire inst_beq;
    wire inst_bne;
    wire inst_blt;
    wire inst_bge;
    wire inst_bltu;
    wire inst_bgeu;
    wire inst_jal;
    wire inst_jalr;


    /*系统指令标签：ecall 触发异常，mret 返回异常，csrr* 读改写 CSR。*/
    wire inst_ecall;   // ecall 指令
    wire inst_mret;    // mret 指令
    wire inst_csrrw;   // csrrw
    wire inst_csrrs;   // csrrs
    wire inst_csrrc;   // csrrc
    wire inst_csrrwi;  // csrrwi
    wire inst_csrrsi;  // csrrsi
    wire inst_csrrci;  // csrrci
 
    /*csr_addr 是 CSR 的地址；csr_uimm 是 CSR 立即数版本使用的 5 位小常数。*/
    wire [11:0] csr_addr;   // 指令[31:20]
    wire [4:0]  csr_uimm;   // 指令[19:15]，用于csrrwi/csrrsi/csrrci


    //============================================================
    // slot0 full instruction decode
    //============================================================
    /*..................internal signals................*/

 


    /*把 32 位机器码拆成几块：opcode 看大类，funct3/funct7 看具体操作，
    rd 是目的寄存器，rs1/rs2 是源寄存器。*/
    wire [ 6:0] opcode = id_inst[ 6: 0]; 
    wire [ 4:0] rd     = id_inst[11: 7]; 
    wire [ 2:0] funct3 = id_inst[14:12]; 
    wire [ 4:0] rs1    = id_inst[19:15]; // 对应原rj
    wire [ 4:0] rs2    = id_inst[24:20]; // 对应原rk
    wire [ 6:0] funct7 = id_inst[31:25]; 

    /*不同指令格式把立即数字段放在不同位置。
    ID 先把 I/S/B/U/J 五种立即数拼成完整 32 位，EXE 直接拿来用。*/
    wire [31:0] imm_I = {{20{id_inst[31]}}, id_inst[31:20]};
    wire [31:0] imm_S = {{20{id_inst[31]}}, id_inst[31:25], id_inst[11:7]};
    wire [31:0] imm_B = {{20{id_inst[31]}}, id_inst[7], id_inst[30:25], id_inst[11:8], 1'b0};
    wire [31:0] imm_U = {id_inst[31:12], 12'b0};
    wire [31:0] imm_J = {{12{id_inst[31]}}, id_inst[19:12], id_inst[20], id_inst[30:21], 1'b0};

    /*先按 opcode 给指令分大类，后面再结合 funct3/funct7 判断具体指令。*/
    wire op_R_TYPE  = (opcode == 7'b0110011); // 寄存器算术运算
    wire op_I_TYPE  = (opcode == 7'b0010011); // 立即数算术运算
    wire op_LOAD    = (opcode == 7'b0000011); // 访存读
    wire op_STORE   = (opcode == 7'b0100011); // 访存写
    wire op_BRANCH  = (opcode == 7'b1100011); // 跳转判定
    wire op_JAL     = (opcode == 7'b1101111); // 无条件直接跳转链接
    wire op_JALR    = (opcode == 7'b1100111); // 无条件间接跳转链接
    wire op_LUI     = (opcode == 7'b0110111); // 高位加载
    wire op_AUIPC   = (opcode == 7'b0010111); // 新增：AUIPC 指令大类wire op_AUIPC   = (opcode == 7'b0010111); // 新增：AUIPC 指令大类
    wire op_SYSTEM  = (opcode == 7'b1110011);// [新增] SYSTEM 大类（ecall/ebreak/mret/CSR指令都用这个opcode）

    /*is_rv32m 表示这是 RV32M 指令；mdu_op 直接使用 funct3，告诉乘除法单元具体做什么。
    八个 inst_mul/inst_div/... 名字只在 ID 里帮助阅读和译码，跨级只带这两项。*/
    wire is_rv32m = inst_mul | inst_mulh | inst_mulhsu | inst_mulhu |
                    inst_div | inst_divu | inst_rem    | inst_remu;
    wire [2:0] mdu_op = funct3;


 
    /*结合 funct3 和 funct7，把大类翻译成具体指令。*/
    wire inst_add   = op_R_TYPE & (funct3 == 3'b000) & (funct7 == 7'b0000000);
    wire inst_sub   = op_R_TYPE & (funct3 == 3'b000) & (funct7 == 7'b0100000); 
    wire inst_sll   = op_R_TYPE & (funct3 == 3'b001) & (funct7 == 7'b0000000); // 新增
    wire inst_slt   = op_R_TYPE & (funct3 == 3'b010) & (funct7 == 7'b0000000);
    wire inst_sltu  = op_R_TYPE & (funct3 == 3'b011) & (funct7 == 7'b0000000);
    wire inst_xor   = op_R_TYPE & (funct3 == 3'b100) & (funct7 == 7'b0000000);
    wire inst_srl   = op_R_TYPE & (funct3 == 3'b101) & (funct7 == 7'b0000000); // 新增
    wire inst_sra   = op_R_TYPE & (funct3 == 3'b101) & (funct7 == 7'b0100000); // 新增
    wire inst_or    = op_R_TYPE & (funct3 == 3'b110) & (funct7 == 7'b0000000);
    wire inst_and   = op_R_TYPE & (funct3 == 3'b111) & (funct7 == 7'b0000000);
    /*I 型整数运算。*/
    wire inst_addi  = op_I_TYPE & (funct3 == 3'b000);
    wire inst_slli  = op_I_TYPE & (funct3 == 3'b001) & (funct7 == 7'b0000000);
    wire inst_slti  = op_I_TYPE & (funct3 == 3'b010); // 新增
    wire inst_sltui = op_I_TYPE & (funct3 == 3'b011); // 新增
    wire inst_xori  = op_I_TYPE & (funct3 == 3'b100); // 新增
    wire inst_srli  = op_I_TYPE & (funct3 == 3'b101) & (funct7 == 7'b0000000);
    wire inst_srai  = op_I_TYPE & (funct3 == 3'b101) & (funct7 == 7'b0100000);
    wire inst_ori   = op_I_TYPE & (funct3 == 3'b110); // 新增
    wire inst_andi  = op_I_TYPE & (funct3 == 3'b111); // 新增
    /*orc.b 的固定编码位很多，必须按完整掩码匹配，不能只看 opcode/funct3/funct7，
    否则其他位操作可能被误认成 orc.b。*/
    wire inst_orcb = ((id_inst & 32'hfff0_707f) == 32'h2870_5013);
    /*load 指令的宽度。*/
    assign inst_lb  = op_LOAD   & (funct3 == 3'b000); // 新增
    assign inst_lh  = op_LOAD   & (funct3 == 3'b001); // 新增
    assign inst_lw  = op_LOAD   & (funct3 == 3'b010);
    assign inst_lbu = op_LOAD   & (funct3 == 3'b100); // 新增
    assign inst_lhu = op_LOAD   & (funct3 == 3'b101); // 新增
    /*store 指令的宽度。*/
    assign inst_sb  = op_STORE  & (funct3 == 3'b000); // 新增
    assign inst_sh  = op_STORE  & (funct3 == 3'b001); // 新增
    assign inst_sw  = op_STORE  & (funct3 == 3'b010);
    /*条件分支和无条件跳转。*/
    assign inst_beq   = op_BRANCH & (funct3 == 3'b000);
    assign inst_bne   = op_BRANCH & (funct3 == 3'b001);
    assign inst_blt   = op_BRANCH & (funct3 == 3'b100); 
    assign inst_bge   = op_BRANCH & (funct3 == 3'b101); 
    assign inst_bltu  = op_BRANCH & (funct3 == 3'b110); 
    assign inst_bgeu  = op_BRANCH & (funct3 == 3'b111); 
    //J型 & I型：无条件跳转
    assign inst_jal   = op_JAL;
    assign inst_jalr  = op_JALR   & (funct3 == 3'b000);
    /*U 型长立即数指令。*/
    wire inst_lui   = op_LUI;
    wire inst_auipc = op_AUIPC; // 新增

    /*SYSTEM 类指令：ecall 和 mret 用完整编码识别。*/
    assign inst_ecall  = op_SYSTEM & (funct3 == 3'b000) & (id_inst[31:20] == 12'b0000_0000_0000);
    assign inst_mret   = op_SYSTEM & (funct3 == 3'b000) & (id_inst[31:20] == 12'b0011_0000_0010);
    /*funct3=001/010/011 是寄存器形式的 CSR 操作。*/
    assign inst_csrrw  = op_SYSTEM & (funct3 == 3'b001);
    assign inst_csrrs  = op_SYSTEM & (funct3 == 3'b010);
    assign inst_csrrc  = op_SYSTEM & (funct3 == 3'b011);
    /*funct3=101/110/111 是立即数形式的 CSR 操作。*/
    assign inst_csrrwi = op_SYSTEM & (funct3 == 3'b101);
    assign inst_csrrsi = op_SYSTEM & (funct3 == 3'b110);
    assign inst_csrrci = op_SYSTEM & (funct3 == 3'b111);
 
    /*RV32M 的八条指令共用 R 型 opcode 和 funct7=0000001。
    保留八个独立名字，波形里可以直接看出是 mul、div 还是 rem。*/
    wire rv32m_encoding = op_R_TYPE & (funct7 == 7'b0000001);
    wire inst_mul       = rv32m_encoding & (funct3 == 3'b000);
    wire inst_mulh      = rv32m_encoding & (funct3 == 3'b001);
    wire inst_mulhsu    = rv32m_encoding & (funct3 == 3'b010);
    wire inst_mulhu     = rv32m_encoding & (funct3 == 3'b011);
    wire inst_div       = rv32m_encoding & (funct3 == 3'b100);
    wire inst_divu      = rv32m_encoding & (funct3 == 3'b101);
    wire inst_rem       = rv32m_encoding & (funct3 == 3'b110);
    wire inst_remu      = rv32m_encoding & (funct3 == 3'b111);


    /*所有 CSR 指令的地址都在指令 [31:20]。*/
    assign csr_addr = id_inst[31:20];
    /*CSR 立即数版本把 rs1 字段 [19:15] 当作 5 位无符号数。*/
    assign csr_uimm = id_inst[19:15];
 
    // is_csr_inst：当前指令是任意一条CSR读写指令。ecall/mret不是CSR读写，所以不算进来。
    wire is_csr_inst = inst_csrrw | inst_csrrs | inst_csrrc |
                       inst_csrrwi | inst_csrrsi | inst_csrrci;
    

    /*下面生成的是“控制按钮”，不是实际数据。
    alu_op 决定 ALU 的运算；src1/src2 选择输入来源；mem_* 描述访存；reg_* 描述写回。*/
    assign alu_op[ 0] = inst_add | inst_addi | op_LOAD | op_STORE | op_JAL | op_JALR | inst_auipc; 
    assign alu_op[ 1] = inst_sub; 
    assign alu_op[ 2] = inst_slt | inst_slti;
    assign alu_op[ 3] = inst_sltu | inst_sltui;
    assign alu_op[ 4] = inst_and | inst_andi;
    /*orc.b 借用原来没有使用的 ALU 控制位，因此总线仍保持 12 位。*/
    assign alu_op[ 5] = inst_orcb;
    assign alu_op[ 6] = inst_or | inst_ori;
    assign alu_op[ 7] = inst_xor | inst_xori;
    assign alu_op[ 8] = inst_slli | inst_sll;
    assign alu_op[ 9] = inst_srli | inst_srl;
    assign alu_op[10] = inst_srai | inst_sra;
    assign alu_op[11] = inst_lui; // 让立即数直通 ALU

    /*src1_is_pc、src2_is_4、src2_is_imm 和 imm 一起决定 ALU 两个输入。
    例如 JAL 要写回 PC+4，就选择 PC 和常数 4；ADDI 则选择 rs1 和 imm_I。*/
    assign src1_is_pc  = op_JAL | op_JALR | inst_auipc; // 修改：增加 auipc
    
    wire src2_is_4     = op_JAL | op_JALR;

    assign src2_is_imm = op_I_TYPE | op_LOAD | op_STORE | op_LUI | op_JAL | op_JALR | op_AUIPC; // 修改：增加 auipc

    assign imm = src2_is_4                       ? 32'h4 : 
                 (op_I_TYPE | op_LOAD | op_JALR) ? imm_I : 
                 (op_STORE)                      ? imm_S : 
                 (op_LUI | op_AUIPC)             ? imm_U : 
                 32'b0;


 
    /*mem_en/mem_we 是给 MEM 的访存工作单；res_from_mem、reg_waddr、reg_we 是给 WB 的写回工作单。
    ID 只负责填写这些信息，真正动作要等指令走到对应阶段。*/
    assign mem_en       = op_LOAD | op_STORE;
    assign mem_we       = {4{op_STORE}};       
    assign res_from_mem = op_LOAD;
    
    assign reg_waddr    = rd;
    assign reg_we       = (op_R_TYPE | op_I_TYPE | op_LOAD | op_LUI | op_JAL | op_JALR | op_AUIPC | is_csr_inst) && id_valid;
    /*CSR 指令写回 rd 的是 CSR 旧值；ecall 和 mret 不写通用寄存器。*/



    /*这里不再比较寄存器里的数据。
    use_rf_rdata1/use_rf_rdata2 只说明指令要不要读 rs1/rs2；
    真正的 RAW、load-use 和前递判断已经放到 ISSUE。*/
    assign use_rf_rdata1 = id_valid && (op_R_TYPE | op_I_TYPE | op_LOAD | op_STORE | op_BRANCH | op_JALR | is_csr_inst); // 只要是这些类型的指令，就需要用到 rs1 的值（即使是 jalr 和 csr 指令，虽然它们的 rs1 可能不参与运算，但它们也需要读寄存器堆来获取 rs1 的值）
    assign use_rf_rdata2 = id_valid && (op_R_TYPE | op_STORE | op_BRANCH);




    //============================================================
    // RAS maintenance back to IF1
    //============================================================
    /*RAS 只服务函数调用和函数返回预测。
    x1(ra) 和 x5(t0) 是 RISC-V 约定的链接寄存器：写入它们通常表示 CALL，
    从它们跳回通常表示 RET。下面的 push/pop 只有在指令真正发射时才生效。*/
    wire is_link_reg_rd  = (rd == 5'd1) || (rd == 5'd5);
    wire is_link_reg_rs1 = (rs1 == 5'd1) || (rs1 == 5'd5);

    wire slot0_push = (inst_jal || inst_jalr) && is_link_reg_rd;
    wire slot0_pop  = inst_jalr && is_link_reg_rs1 && (rd != rs1);
    wire [2:0] id_safe_ras_ptr = slot0_push ?
                                 (current_ras_ptr + 3'b1) : current_ras_ptr;

    wire id_is_ret = inst_jalr && is_link_reg_rs1 && (rd != rs1);
    /*mret 返回的是 CSR mepc，不是函数返回，不能操作 RAS。
    slot0_push 表示把 PC+4 压入 RAS；slot0_pop 表示从链接寄存器返回。*/
    wire id_fire = id_to_issue_valid && issue_allow_in;
    wire id_push_ras = id_fire && (slot0_push || lane1_push);

    wire id_pop_ras  = id_fire && (slot0_pop || lane1_pop);

    wire [31:0] id_ras_wdata = lane1_push ? shadow_pc + 32'd4 : id_pc + 32'd4;

    /*把两个槽合并成一组 RAS 动作送回 IF。
    如果 slot1 是 CALL，它更年轻，所以压入的返回地址优先使用 slot1 的 PC+4。*/
    assign id_to_if_bus = {id_push_ras, id_pop_ras, id_ras_wdata};

    //============================================================
    // decoded control buses to ISSUE
    //============================================================

    wire [`ID_DATA_BUS_WIDTH-1:0] slot0_bus;
    assign slot0_bus = {
        id_pc,
        id_inst,
        alu_op,
        src1_is_pc,
        src2_is_imm,
        res_from_mem,
        mem_en,
        mem_we,
        reg_we,
        reg_waddr,
        rs1_value,
        rs2_value,
        imm,
        inst_lb,  // 新增
        inst_lh,  // 新增
        inst_lw,  // 新增
        inst_lbu, // 新增
        inst_lhu, // 新增
        inst_sb,  // 新增
        inst_sh,  // 新增
        inst_sw,  // 新增      

        inst_beq,
        inst_bne,
        inst_blt,
        inst_bge,
        inst_bltu,
        inst_bgeu,
        inst_jal,
        inst_jalr,
        pre_taken, pre_target, pre_index,
        imm_B, imm_J, imm_I,
        id_is_ret,
        id_safe_ras_ptr,

        // CSR 控制字段
        inst_ecall, inst_mret,
        inst_csrrw, inst_csrrs, inst_csrrc,
        inst_csrrwi, inst_csrrsi, inst_csrrci,
        csr_addr,    // 12位
        csr_uimm,     // 5位
        is_csr_inst,

        // RV32M 只传类别位和原始 funct3，后面的 MDU 直接使用 funct3 解码。
        is_rv32m,
        mdu_op
    };
    /*slot0 没有同组前一条 ALU 结果可依赖，所以最高端的三个 late/dep 位固定为 0。*/
    assign id_to_issue_bus = {3'b000, slot0_bus};
    // 总线位宽由 mycpu_top.h 统一定义；这里保持字段顺序不变。





    /*CSR 写入要到 WB 才真正改变状态。
    如果 EXE/MEM/WB 还有 CSR 写或 mret，而 ID 又要读取相关 CSR，ID 可能看到旧值。
    所以这里用两个 4 位移位寄存器记住在途指令；只要还有一位为 1，就暂时停住 ID。
    mret 最终在 WB 产生 mret_flush，PC 重定向由 IF 完成。*/
 
    /*不改各级总线，直接在 ID 记录指令进入后端的 4 个位置：
    bit0 表示 ISSUE，bit1 表示 EXE，bit2 表示 MEM，bit3 表示 WB。
    每拍整体左移一位，用固定的 4 拍窗口保守地覆盖 CSR 写和 mret。*/
 
    reg [3:0] csr_write_in_pipe;
    reg [3:0] mret_in_pipe;
 
    wire id_is_csr_write = is_csr_inst && id_valid && id_ready_go && !wb_flush && !flush_en;
    wire id_is_mret      = inst_mret   && id_valid && id_ready_go && !wb_flush && !flush_en;
 
    always @(posedge clk) begin
        if (reset || wb_flush) begin
            csr_write_in_pipe <= 4'b0;
            mret_in_pipe      <= 4'b0;
        end else begin
            /*每拍把旧记录向 WB 方向推一格，并在 bit0 放入当前离开 ID 的指令。
            这是保守跟踪，多停一拍只影响性能，不改变结果。*/
            csr_write_in_pipe <= {csr_write_in_pipe[2:0], id_is_csr_write};
            mret_in_pipe      <= {mret_in_pipe[2:0],      id_is_mret};
        end
    end
 
    /*任意在途位置还有 CSR 写或 mret，ID 就不能继续读取相关状态。*/
    wire csr_hazard = (|csr_write_in_pipe) || (|mret_in_pipe);

    
    assign id_ready_go = !csr_hazard;
     /*id_allow_in = ~id_valid || id_ready_go && exe_allow_in=0,
    从而使if_allow_in = !if_valid || (if_ready_go && id_allow_in)=0，
    always @(posedge clk) begin
        if (reset) begin
            if_valid <= 1'b0;
        end else if (if_allow_in) begin                
            if_valid <= ~reset;
        end
    end

    always@ (posedge clk) begin
        if (reset) begin
            if_pc <= 32'h1bfffffc;
        end else if ( if_allow_in) begin
            if_pc <= nextpc;//相当于其他阶段的reg
        end
    end
     从而使if_stage的指令也停住了，if_pc也不更新了，这样就不会有新的指令进入if_stage了                                   
    */


    
    
    /*我要产生一个信号 br_cancel，用来在时钟上升沿把 ID 阶段清空为气泡（id_valid <= 0）
    1、什么时候应该清空 ID？
    肯定是当目前 ID 阶段的指令是一条分支指令，并且它算出了真正要跳（br_taken = 1）。
    此外，前提是现在 ID 阶段真的有指令，不能是个空壳气泡（id_valid = 1）
    assign br_cancel = id_valid && br_taken就写出来了

    2、如果在 ID 阶段的这条指令是 beq，它确实要跳（id_valid=1, br_taken=1），
    那么在这个时钟周期内，br_cancel 就会一直高高地挂在 1
    假设出现了一种意外：前面有一条 load 指令还没把数据取回来（load-use 暂停）
    对于 beq 来说：它不能前进，它必须在下一个时钟周期继续留在 ID 阶段。
    可怕的事情来了：在这个时钟周期的末尾（上升沿到来时），电路会去检查触发器
        always @(posedge clk) begin
        if (reset) begin ...
        end else if (br_cancel) begin // 此时 br_cancel = 1 !!!!!
            id_valid <= 1'b0;         // ID 阶段被清空了！
        end ...

        beq 本来因为缺数据，正在乖乖排队等待（Stall）。
    结果因为提前拉高了 br_cancel，在时钟上升沿，beq 居然把自己给清空（自杀）了！
    它连 EXE 阶段都没进去，直接灰飞烟灭。指令凭空消失，CPU 死机跑飞 
    为了防止 beq 停顿的时候自杀，我们必须给 br_cancel 加上一把**“时限锁”**。

    3、在极其复杂的流水线中，所有动作的黄金准则是：
    “绝不能提前改变状态，只能在你成功离开当前阶段的最后一刻，触发副作用。”
    怎么判断“我马上就要离开当前阶段了”？
    看 ready_go 信号。
    id_ready_go == 0：我被卡住了，我还不准备走，一切向别人发送的“副作用”动作都必须憋住。
    id_ready_go == 1：本阶段计算全部完成，没有暂停，我已经打包好行李，下个时钟沿必定滚蛋。    
    */




endmodule








//============================================================
// local dual-issue pairing control
//============================================================

/*本文件私有辅助模块：dual_issue_ctrl 检查两条指令能不能同拍发射。
它只输出“允许配对”或“为什么不能配对”，不直接改变 FIFO 或流水线状态。*/
`timescale 1ns / 1ps

module dual_issue_ctrl (
    input  wire        valid0,
    input  wire        valid1,
    input  wire [24:0] dec0,
    input  wire [24:0] dec1,
    output reg         pair_ok,
    output reg         late,
    output reg         block_raw,
    output reg         block_waw,
    output reg         block_lsu,
    output reg         block_mul,
    output reg         block_ctrl,
    output reg         block_serial
);
    wire legal0  = dec0[24];
    wire rs1_en1 = dec1[23];
    wire rs2_en1 = dec1[22];
    wire rd_we0  = dec0[21];
    wire rd_we1  = dec1[21];
    wire alu0    = dec0[20];
    wire alu1    = dec1[20];
    wire lsu0    = dec0[19];
    wire lsu1    = dec1[19];
    wire mul0    = dec0[18];
    wire mul1    = dec1[18];
    wire branch0 = dec0[16];
    wire serial0 = dec0[15];
    wire serial1 = dec1[15];
    wire [4:0] rs1_1 = dec1[14:10];
    wire [4:0] rs2_1 = dec1[9:5];
    wire [4:0] rd0   = dec0[4:0];
    wire [4:0] rd1   = dec1[4:0];

    wire attempt = valid0 && valid1;
    wire raw = rd_we0 && (rd0 != 5'b0) &&
               ((rs1_en1 && (rs1_1 == rd0)) ||
                (rs2_en1 && (rs2_1 == rd0)));
    wire waw = rd_we0 && rd_we1 && (rd0 != 5'b0) && (rd0 == rd1);
    wire two_lsu = lsu0 && lsu1;
    wire two_mul = mul0 && mul1;
    /*只有普通 ALU+普通 ALU 的 RAW 可以用后置 ALU 解决。
    load、MUL 或其他晚结果不能马上给 slot1，所以仍然阻塞。*/
    wire late_raw = raw && alu0 && alu1;
    wire allow_waw = waw && alu0 && alu1;

    /*slot0 仍然不能和任何年轻指令一起发射控制流指令。
    slot1 的分支、跳转有自己的比较器和目标地址通路，因此可以跟随
    slot0 的 ALU、LSU 或 MUL；这里只把真正共享的资源交给下面的
    two_lsu、two_mul 检查，而不是按指令类别一概禁止配对。*/
    wire ctrl = branch0;
    wire serial = serial0 || serial1 || !legal0 || !dec1[24];

    /*多个问题同时存在时，只报告一个原因，优先级固定为：
    串行/非法、控制流位置、RAW、WAW、双 LSU、双 MUL。*/
    always @(*) begin
        pair_ok = 1'b0;
        late = 1'b0;
        block_raw = 1'b0;
        block_waw = 1'b0;
        block_lsu = 1'b0;
        block_mul = 1'b0;
        block_ctrl = 1'b0;
        block_serial = 1'b0;

        if (attempt) begin
            if (serial)
                block_serial = 1'b1;
            else if (ctrl)
                block_ctrl = 1'b1;
            else if (raw && !late_raw)
                block_raw = 1'b1;
            else if (waw && !allow_waw)
                block_waw = 1'b1;
            else if (two_lsu)
                block_lsu = 1'b1;
            else if (two_mul)
                block_mul = 1'b1;
            else begin
                pair_ok = 1'b1;
                late = late_raw;
            end
        end
    end
endmodule








//============================================================
// local slot decoder
//============================================================

/*本文件私有辅助模块：id_lane 是 slot1 使用的完整译码器。
它生成和 slot0 完全相同格式的控制包，这样 slot1 可以不改接口地穿过 ISSUE、EXE、MEM、WB。*/
`include "mycpu_top.h"

module id_lane(
    input  wire        valid,
    input  wire [31:0] pc,
    input  wire [31:0] inst,
    input  wire        pre_taken,
    input  wire [31:0] pre_target,
    input  wire [5:0]  pre_index,
    input  wire [2:0]  ras_ptr,
    output wire [`ID_DATA_BUS_WIDTH-1:0] bus,
    output wire [11:0] rf_bus,
    output wire        push_ras,
    output wire        pop_ras
);
    /*slot1 也按同样顺序拆机器码，保证它和 slot0 使用相同的译码思路。*/
    wire [6:0] opcode = inst[6:0];
    wire [4:0] rd     = inst[11:7];
    wire [2:0] funct3 = inst[14:12];
    wire [4:0] rs1    = inst[19:15];
    wire [4:0] rs2    = inst[24:20];
    wire [6:0] funct7 = inst[31:25];

    /*op_* 是大类标签；inst_* 是具体指令标签。后面据此生成 ALU、访存和 RAS 控制。*/
    wire op_r      = opcode == 7'b0110011;
    wire op_i      = opcode == 7'b0010011;
    wire op_load   = opcode == 7'b0000011;
    wire op_store  = opcode == 7'b0100011;
    wire op_branch = opcode == 7'b1100011;
    wire op_jal    = opcode == 7'b1101111;
    wire op_jalr   = opcode == 7'b1100111;
    wire op_lui    = opcode == 7'b0110111;
    wire op_auipc  = opcode == 7'b0010111;
    wire op_system = opcode == 7'b1110011;

    wire rv32m = op_r && funct7 == 7'b0000001;
    wire inst_add  = op_r && funct3 == 3'b000 && funct7 == 7'b0000000;
    wire inst_sub  = op_r && funct3 == 3'b000 && funct7 == 7'b0100000;
    wire inst_sll  = op_r && funct3 == 3'b001 && funct7 == 7'b0000000;
    wire inst_slt  = op_r && funct3 == 3'b010 && funct7 == 7'b0000000;
    wire inst_sltu = op_r && funct3 == 3'b011 && funct7 == 7'b0000000;
    wire inst_xor  = op_r && funct3 == 3'b100 && funct7 == 7'b0000000;
    wire inst_srl  = op_r && funct3 == 3'b101 && funct7 == 7'b0000000;
    wire inst_sra  = op_r && funct3 == 3'b101 && funct7 == 7'b0100000;
    wire inst_or   = op_r && funct3 == 3'b110 && funct7 == 7'b0000000;
    wire inst_and  = op_r && funct3 == 3'b111 && funct7 == 7'b0000000;

    wire inst_addi  = op_i && funct3 == 3'b000;
    wire inst_slli  = op_i && funct3 == 3'b001 && funct7 == 7'b0000000;
    wire inst_slti  = op_i && funct3 == 3'b010;
    wire inst_sltiu = op_i && funct3 == 3'b011;
    wire inst_xori  = op_i && funct3 == 3'b100;
    wire inst_srli  = op_i && funct3 == 3'b101 && funct7 == 7'b0000000;
    wire inst_srai  = op_i && funct3 == 3'b101 && funct7 == 7'b0100000;
    wire inst_ori   = op_i && funct3 == 3'b110;
    wire inst_andi  = op_i && funct3 == 3'b111;
    /*slot1 的 orc.b 也按完整固定编码识别，不能放宽条件。*/
    wire inst_orcb = ((inst & 32'hfff0_707f) == 32'h2870_5013);

    wire inst_lb  = op_load && funct3 == 3'b000;
    wire inst_lh  = op_load && funct3 == 3'b001;
    wire inst_lw  = op_load && funct3 == 3'b010;
    wire inst_lbu = op_load && funct3 == 3'b100;
    wire inst_lhu = op_load && funct3 == 3'b101;
    wire inst_sb  = op_store && funct3 == 3'b000;
    wire inst_sh  = op_store && funct3 == 3'b001;
    wire inst_sw  = op_store && funct3 == 3'b010;

    wire inst_beq  = op_branch && funct3 == 3'b000;
    wire inst_bne  = op_branch && funct3 == 3'b001;
    wire inst_blt  = op_branch && funct3 == 3'b100;
    wire inst_bge  = op_branch && funct3 == 3'b101;
    wire inst_bltu = op_branch && funct3 == 3'b110;
    wire inst_bgeu = op_branch && funct3 == 3'b111;
    wire inst_jal  = op_jal;
    wire inst_jalr = op_jalr && funct3 == 3'b000;

    wire inst_ecall = inst == 32'h0000_0073;
    wire inst_mret  = inst == 32'h3020_0073;
    wire inst_csrrw  = op_system && funct3 == 3'b001;
    wire inst_csrrs  = op_system && funct3 == 3'b010;
    wire inst_csrrc  = op_system && funct3 == 3'b011;
    wire inst_csrrwi = op_system && funct3 == 3'b101;
    wire inst_csrrsi = op_system && funct3 == 3'b110;
    wire inst_csrrci = op_system && funct3 == 3'b111;
    wire csr_inst = inst_csrrw | inst_csrrs | inst_csrrc |
                    inst_csrrwi | inst_csrrsi | inst_csrrci;

    /*slot1 必须自己拼立即数，才能生成和 slot0 完全相同的后端控制包。*/
    wire [31:0] imm_i = {{20{inst[31]}}, inst[31:20]};
    wire [31:0] imm_s = {{20{inst[31]}}, inst[31:25], inst[11:7]};
    wire [31:0] imm_b = {{20{inst[31]}}, inst[7], inst[30:25],
                         inst[11:8], 1'b0};
    wire [31:0] imm_u = {inst[31:12], 12'b0};
    wire [31:0] imm_j = {{12{inst[31]}}, inst[19:12], inst[20],
                         inst[30:21], 1'b0};

    wire src1_pc = inst_jal | inst_jalr | op_auipc;
    wire src2_imm = op_i | op_load | op_store | op_lui |
                    inst_jal | inst_jalr | op_auipc;
    wire [31:0] imm = (inst_jal | inst_jalr) ? 32'd4 :
                      (op_i | op_load | inst_jalr) ? imm_i :
                      op_store ? imm_s :
                      (op_lui | op_auipc) ? imm_u : 32'b0;

    /*src1_pc/src2_imm 选择 ALU 两个输入；alu_op 选择具体运算。*/
    wire [11:0] alu_op;
    assign alu_op[0]  = inst_add | inst_addi | op_load | op_store |
                        inst_jal | inst_jalr | op_auipc;
    assign alu_op[1]  = inst_sub;
    assign alu_op[2]  = inst_slt | inst_slti;
    assign alu_op[3]  = inst_sltu | inst_sltiu;
    assign alu_op[4]  = inst_and | inst_andi;
    /*orc.b 复用原来空闲的 ALU 控制位，不增加总线宽度。*/
    assign alu_op[5]  = inst_orcb;
    assign alu_op[6]  = inst_or | inst_ori;
    assign alu_op[7]  = inst_xor | inst_xori;
    assign alu_op[8]  = inst_sll | inst_slli;
    assign alu_op[9]  = inst_srl | inst_srli;
    assign alu_op[10] = inst_sra | inst_srai;
    assign alu_op[11] = op_lui;

    /*rf_bus 只送源寄存器编号和使用标志，寄存器的实际数值由 ISSUE 读取。
    push_ras/pop_ras 是这条 slot1 指令对返回地址栈的请求。*/
    wire use_rs1 = valid && (op_r | op_i | op_load | op_store |
                             op_branch | inst_jalr |
                             inst_csrrw | inst_csrrs | inst_csrrc);
    wire use_rs2 = valid && (op_r | op_store | op_branch);
    wire reg_we = valid && (op_r | op_i | op_load | op_lui | inst_jal |
                            inst_jalr | op_auipc | csr_inst);
    assign rf_bus = {use_rs1, use_rs2, rs1, rs2};

    wire link_rd = rd == 5'd1 || rd == 5'd5;
    wire link_rs1 = rs1 == 5'd1 || rs1 == 5'd5;
    assign push_ras = valid && (inst_jal | inst_jalr) && link_rd;
    assign pop_ras  = valid && inst_jalr && link_rs1 && rd != rs1;
    wire is_ret = inst_jalr && link_rs1 && rd != rs1;
    wire [2:0] safe_ras_ptr = push_ras ? ras_ptr + 3'd1 : ras_ptr;

    /*slot1 的字段顺序和 slot0 完全一致。
    两个操作数位置先填 0，ISSUE 会用四读寄存器堆和前递结果补上真正的值。*/
    assign bus = {
        pc, inst, alu_op, src1_pc, src2_imm,
        op_load, op_load | op_store, {4{op_store}}, reg_we, rd,
        32'b0, 32'b0, imm,
        inst_lb, inst_lh, inst_lw, inst_lbu, inst_lhu,
        inst_sb, inst_sh, inst_sw,
        inst_beq, inst_bne, inst_blt, inst_bge,
        inst_bltu, inst_bgeu, inst_jal, inst_jalr,
        pre_taken, pre_target, pre_index,
        imm_b, imm_j, imm_i, is_ret, safe_ras_ptr,
        inst_ecall, inst_mret,
        inst_csrrw, inst_csrrs, inst_csrrc,
        inst_csrrwi, inst_csrrsi, inst_csrrci,
        inst[31:20], inst[19:15], csr_inst,
        rv32m, funct3
    };
endmodule

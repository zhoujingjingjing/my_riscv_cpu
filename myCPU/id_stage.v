`include "mycpu_top.h"
module id_stage(
    input wire clk,
    input wire reset,

    output wire [`ID_TO_EXE_BUS_WIDTH-1:0] id_to_exe_bus,//bus to EXE stage
    output wire [`ID_TO_IF_BUS_WIDTH-1:0]  id_to_if_bus, //bus to IF stage (for branch)
    input  wire [`IF_TO_ID_BUS_WIDTH-1:0]  if_to_id_bus, //bus from IF stage
    input wire  [`WB_TO_ID_BUS_WIDTH-1:0]  wb_to_id_bus, //bus from WB stage(for regfile and bypass)
    input wire  [`EXE_TO_ID_BYPASS_BUS_WIDTH-1:0]    exe_to_id_bypass_bus, //bus from EXE stage for bypass
    input wire  [`MEM_TO_ID_BYPASS_BUS_WIDTH-1:0]    mem_to_id_bypass_bus, //bus from MEM stage for bypass

  
    output wire id_allow_in,
    output wire id_to_exe_valid,
    input wire exe_allow_in,
    input wire if_to_id_valid

);


    /*.........pipeline control.............*/
    
    wire id_ready_go;
    reg  id_valid;
    reg [`IF_TO_ID_BUS_WIDTH-1:0] id_reg;
    wire br_cancel;
    always @ (posedge clk) begin
        if (reset) begin
            id_valid <= 1'b0;
        end else if (br_cancel) begin
            id_valid <= 1'b0;//如果分支跳转了，那么就把id_stage的指令作废了，变成气泡    
        end else if (id_allow_in) begin
            id_valid <= if_to_id_valid;
        end
    end

    // assign id_ready_go = 1'b1;
    assign id_allow_in = ~id_valid || id_ready_go && exe_allow_in;
    assign id_to_exe_valid = id_valid && id_ready_go;

  /*............input bus from IF stage.............*/
    always @(posedge clk) begin
        if (if_to_id_valid && id_allow_in) begin
            id_reg <= if_to_id_bus;
        end
    end

    wire  [31:0] id_pc;
    wire [31:0] id_inst;
    assign {id_pc, id_inst} = id_reg;//输入


    /*..........input bus from WB stage..............*/
    wire wb_valid;
    wire wb_rf_we;
    wire [4:0] wb_rf_waddr;
    wire [31:0] wb_rf_wdata;
    assign {wb_valid, wb_rf_we, wb_rf_waddr, wb_rf_wdata} = wb_to_id_bus;//输入
  
    /*..........input bus from exe stage..............*/
    wire exe_valid;
    wire exe_rf_we;
    wire [4:0] exe_rf_waddr;
    wire [31:0] exe_rf_wdata;
    wire exe_is_load;//exe阶段的指令是否是load指令，这个信号是为了在ID阶段判断是否有load-use冒险
    assign {exe_valid, exe_rf_we, exe_rf_waddr, exe_rf_wdata, exe_is_load} = exe_to_id_bypass_bus;

    /*..........input bus from mem stage..............*/
    wire mem_valid;
    wire mem_rf_we;
    wire [4:0] mem_rf_waddr;
    wire [31:0] mem_rf_wdata;
    assign {mem_valid, mem_rf_we, mem_rf_waddr, mem_rf_wdata} = mem_to_id_bypass_bus;


    /*...........output bus to if stage.............*/
    wire        br_taken;
    wire [31:0] br_target;
    assign id_to_if_bus = {br_taken, br_target};//输出


    /*............output bus to EXE stage.............*/

    wire [11:0] alu_op;//控制信号，表示ALU的操作类型，新增
    wire        src1_is_pc;//控制信号，ALU的src1是pc还是RD1，新增
    wire        src2_is_imm;//控制信号4，ALU的src2是立即数还是RD2
    wire        res_from_mem;//控制信号，要写回refile的结果来自mem还是ALU
    wire        mem_en;//控制信号，mem使能，优先于写使能
    wire [3:0]  mem_we;//控制信号，mem写使能，位宽改为4位
    wire        reg_we;//控制信号，refile写使能(写回阶段)。ID阶段产生，但这都是本指令WB阶段才用的上的，现在不能用，得传到下一个阶段，当它自己到了WB阶段再用
    wire [4:0]  reg_waddr;//写回阶段要写回的寄存器地址，bl指令写回r1，其他指令写回rd。ID阶段产生，但这都是本指令WB阶段才用的上的，现在不能用，得传到下一个阶段
    wire [31:0] rs1_value;
    wire [31:0] rs2_value;//选rd还是rk
    wire [31:0] imm;//ALU的src2的立即数值

    // 新增：通过总线传递到后续阶段的具体访存指令信号
    wire        inst_lb;
    wire        inst_lh;
    wire        inst_lw;
    wire        inst_lbu;
    wire        inst_lhu;
    wire        inst_sb;
    wire        inst_sh;
    wire        inst_sw;

    assign id_to_exe_bus = {
        id_pc,
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
        inst_sw   // 新增           
    };
//位宽: 32 + 12 + 1 + 1 + 1 + 1 + 4 + 1 + 5 + 32 + 32 + 32 +8 = 162


    /*..................internal signals................*/

 


    /*拆解指令*/
    wire [ 6:0] opcode = id_inst[ 6: 0]; 
    wire [ 4:0] rd     = id_inst[11: 7]; 
    wire [ 2:0] funct3 = id_inst[14:12]; 
    wire [ 4:0] rs1    = id_inst[19:15]; // 对应原rj
    wire [ 4:0] rs2    = id_inst[24:20]; // 对应原rk
    wire [ 6:0] funct7 = id_inst[31:25]; 

    wire [31:0] imm_I = {{20{id_inst[31]}}, id_inst[31:20]};
    wire [31:0] imm_S = {{20{id_inst[31]}}, id_inst[31:25], id_inst[11:7]};
    wire [31:0] imm_B = {{20{id_inst[31]}}, id_inst[7], id_inst[30:25], id_inst[11:8], 1'b0};
    wire [31:0] imm_U = {id_inst[31:12], 12'b0};
    wire [31:0] imm_J = {{12{id_inst[31]}}, id_inst[19:12], id_inst[20], id_inst[30:21], 1'b0};

    /*完成指令译码，生成指令类型信号*/
    //根据 Opcode 翻译出核心指令大类
    wire op_R_TYPE  = (opcode == 7'b0110011); // 寄存器算术运算
    wire op_I_TYPE  = (opcode == 7'b0010011); // 立即数算术运算
    wire op_LOAD    = (opcode == 7'b0000011); // 访存读
    wire op_STORE   = (opcode == 7'b0100011); // 访存写
    wire op_BRANCH  = (opcode == 7'b1100011); // 跳转判定
    wire op_JAL     = (opcode == 7'b1101111); // 无条件直接跳转链接
    wire op_JALR    = (opcode == 7'b1100111); // 无条件间接跳转链接
    wire op_LUI     = (opcode == 7'b0110111); // 高位加载
    wire op_AUIPC   = (opcode == 7'b0010111); // 新增：AUIPC 指令大类wire op_AUIPC   = (opcode == 7'b0010111); // 新增：AUIPC 指令大类

    //结合 funct3 和 funct7 翻译出具体指令
    // R型
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
    // I型
    wire inst_addi  = op_I_TYPE & (funct3 == 3'b000);
    wire inst_slli  = op_I_TYPE & (funct3 == 3'b001) & (funct7 == 7'b0000000);
    wire inst_slti  = op_I_TYPE & (funct3 == 3'b010); // 新增
    wire inst_sltui = op_I_TYPE & (funct3 == 3'b011); // 新增
    wire inst_xori  = op_I_TYPE & (funct3 == 3'b100); // 新增
    wire inst_srli  = op_I_TYPE & (funct3 == 3'b101) & (funct7 == 7'b0000000);
    wire inst_srai  = op_I_TYPE & (funct3 == 3'b101) & (funct7 == 7'b0100000);
    wire inst_ori   = op_I_TYPE & (funct3 == 3'b110); // 新增
    wire inst_andi  = op_I_TYPE & (funct3 == 3'b111); // 新增
    // I型：内存加载
    assign inst_lb  = op_LOAD   & (funct3 == 3'b000); // 新增
    assign inst_lh  = op_LOAD   & (funct3 == 3'b001); // 新增
    assign inst_lw  = op_LOAD   & (funct3 == 3'b010);
    assign inst_lbu = op_LOAD   & (funct3 == 3'b100); // 新增
    assign inst_lhu = op_LOAD   & (funct3 == 3'b101); // 新增
    // S型：内存存储
    assign inst_sb  = op_STORE  & (funct3 == 3'b000); // 新增
    assign inst_sh  = op_STORE  & (funct3 == 3'b001); // 新增
    assign inst_sw  = op_STORE  & (funct3 == 3'b010);
    // B型：条件分支
    wire inst_beq   = op_BRANCH & (funct3 == 3'b000);
    wire inst_bne   = op_BRANCH & (funct3 == 3'b001);
    wire inst_blt   = op_BRANCH & (funct3 == 3'b100); // 新增
    wire inst_bge   = op_BRANCH & (funct3 == 3'b101); // 新增
    wire inst_bltu  = op_BRANCH & (funct3 == 3'b110); // 新增
    wire inst_bgeu  = op_BRANCH & (funct3 == 3'b111); // 新增
    //J型 & I型：无条件跳转
    wire inst_jal   = op_JAL;
    wire inst_jalr  = op_JALR   & (funct3 == 3'b000);
    //U型：长立即数
    wire inst_lui   = op_LUI;
    wire inst_auipc = op_AUIPC; // 新增

    

    /*控制信号生成*/

    // 控制信号：表示ALU的操作类型 (修改：包含新增指令)
    assign alu_op[ 0] = inst_add | inst_addi | op_LOAD | op_STORE | op_JAL | op_JALR | inst_auipc; 
    assign alu_op[ 1] = inst_sub; 
    assign alu_op[ 2] = inst_slt | inst_slti;
    assign alu_op[ 3] = inst_sltu | inst_sltui;
    assign alu_op[ 4] = inst_and | inst_andi;
    assign alu_op[ 5] = 1'b0; // RISC-V 原生无 nor 指令，这位置 0，不破坏 ALU 原接口
    assign alu_op[ 6] = inst_or | inst_ori;
    assign alu_op[ 7] = inst_xor | inst_xori;
    assign alu_op[ 8] = inst_slli | inst_sll;
    assign alu_op[ 9] = inst_srli | inst_srl;
    assign alu_op[10] = inst_srai | inst_sra;
    assign alu_op[11] = inst_lui; // 让立即数直通 ALU

    // 控制信号：ALU的src1是pc还是rs1
    // JAL / JALR 在 RISC-V 中目的寄存器保存的是 PC+4，所以需要拉 PC 过来借 ALU 相加
    assign src1_is_pc  = op_JAL | op_JALR | inst_auipc; // 修改：增加 auipc
    
    // 配合 src1_is_pc，当算 PC+4 时要送一个 4 进去；否则送解析好的立即数
    wire src2_is_4     = op_JAL | op_JALR;

    // 除 R-Type 和 Branch 外，其余几乎都有立即数(包括算PC+4的)
    assign src2_is_imm = op_I_TYPE | op_LOAD | op_STORE | op_LUI | op_JAL | op_JALR | op_AUIPC; // 修改：增加 auipc

    // 立即数选择
    assign imm = src2_is_4                       ? 32'h4 : 
                 (op_I_TYPE | op_LOAD | op_JALR) ? imm_I : 
                 (op_STORE)                      ? imm_S : 
                 (op_LUI | op_AUIPC)             ? imm_U : 
                 32'b0;


 
    //寄存器堆
    wire [ 4:0] rf_raddr1;
    wire [31:0] rf_rdata1;
    wire [ 4:0] rf_raddr2;
    wire [31:0] rf_rdata2;
    wire        rf_we   ;
    wire [ 4:0] rf_waddr;
    wire [31:0] rf_wdata;

    
    assign rf_raddr1 = rs1;
    assign rf_raddr2 = rs2; 

    regfile u_regfile(
        .clk    (clk      ),
        .raddr1 (rf_raddr1),
        .rdata1 (rf_rdata1),
        .raddr2 (rf_raddr2),
        .rdata2 (rf_rdata2),
        .we     (rf_we    ),
        .waddr  (rf_waddr ),
        .wdata  (rf_wdata )
    );
    
    // 写回
    assign rf_we    = wb_rf_we;   
    assign rf_waddr = wb_rf_waddr;
    assign rf_wdata = wb_rf_wdata;

    //读数
    // assign rs1_value  = rf_rdata1;
    // assign rs2_value = rf_rdata2;
    //rs1_value和rs2_value的值要考虑前递的情况，具体实现在后面



    //分支跳转br unit  
    wire rs1_eq_rs2 = (rs1_value == rs2_value);
    // 新增：提取判断条件给新型分支指令用
    wire rs1_l_rs2  = ($signed(rs1_value) < $signed(rs2_value));     // blt
    wire rs1_lu_rs2 = (rs1_value < rs2_value);                       // bltu
    
    // 判断是否分支发生 (修改：增加大小相关的分支判定)
    assign br_taken = (  (inst_beq  && rs1_eq_rs2)
                      || (inst_bne  && !rs1_eq_rs2)
                      || (inst_blt  && rs1_l_rs2)
                      || (inst_bge  && !rs1_l_rs2)
                      || (inst_bltu && rs1_lu_rs2)
                      || (inst_bgeu && !rs1_lu_rs2)
                      || op_JAL
                      || op_JALR 
                      ) && id_valid;

    // 分支目标地址计算
    // JALR是寄存器相对跳转(需要把计算结果的第0位设为0)；B和J都是PC相对跳转
    assign br_target = (op_BRANCH) ? (id_pc + imm_B) :
                       (op_JAL)    ? (id_pc + imm_J) :
                       (op_JALR)   ? ((rs1_value + imm_I) & ~32'b1) : 
                       32'b0;

    // mem+wb 信号生成
    assign mem_en       = op_LOAD | op_STORE;
    assign mem_we       = {4{op_STORE}};       
    assign res_from_mem = op_LOAD;
    
    // 写寄存器地址永远在 rd 字段
    assign reg_waddr    = rd;
    assign reg_we       = (op_R_TYPE | op_I_TYPE | op_LOAD | op_LUI | op_JAL | op_JALR | op_AUIPC) && id_valid;


    // hazard detection

    /*参与比较的指令到底有没有来自寄存器堆的源操作数，如果没有，就不需要关心冒险了
    add.w r1, r2,r3
    sub.w r4, r1,r5
        有冒险
    即使有，但寄存器号为0，那么也不用进行比较，因为不会对r0进行写操作，所以也不会有冒险
    add.w r0, r2,r3
    sub.w r4, r0,r5
        没有冒险
    */
   // 判断当前译出指令是否需要读对应寄存器？
    wire use_rf_rdata1 = id_valid && (op_R_TYPE | op_I_TYPE | op_LOAD | op_STORE | op_BRANCH | op_JALR);
    wire use_rf_rdata2 = id_valid && (op_R_TYPE | op_STORE | op_BRANCH);

    

    //bypass
    assign rs1_value  = (exe_valid && exe_rf_we && (exe_rf_waddr != 0) && (exe_rf_waddr == rf_raddr1)) ? exe_rf_wdata: //EXE阶段的指令要写回寄存器，并且它写回的寄存器地址不为0，并且它写回的寄存器地址和当前指令读rdata1的寄存器地址相同，那么就有冒险，要前递，前递来自EXE阶段
                       (mem_valid && mem_rf_we && (mem_rf_waddr != 0) && (mem_rf_waddr == rf_raddr1)) ? mem_rf_wdata: //MEM阶段
                       (wb_valid  && wb_rf_we  && (wb_rf_waddr != 0)  && (wb_rf_waddr == rf_raddr1))  ? wb_rf_wdata: //WB阶段
                        rf_rdata1;//正常读寄存器堆的值
   
    assign rs2_value = (exe_valid && exe_rf_we && (exe_rf_waddr != 0) && (exe_rf_waddr == rf_raddr2)) ? exe_rf_wdata: //EXE阶段的指令要写回寄存器，并且它写回的寄存器地址不为0，并且它写回的寄存器地址和当前指令读rdata2的寄存器地址相同，那么就有冒险，要前递，前递来自EXE阶段
                       (mem_valid && mem_rf_we && (mem_rf_waddr != 0) && (mem_rf_waddr == rf_raddr2)) ? mem_rf_wdata: //MEM阶段
                       (wb_valid  && wb_rf_we  && (wb_rf_waddr != 0)  && (wb_rf_waddr == rf_raddr2))  ? wb_rf_wdata: //WB阶段
                        rf_rdata2;
    /*那 4 个条件必须留着： 因为咱们不能拿气泡、或者不写寄存器的脏数据去污染 rs1_value（万一这是一条真的要用 rs1_value 的指令，被脏数据污染就全算错了）。
    但 use_rf_rdata1 可以不留： 因为对于一条本来就不用 rs1_value 的指令来说，把它污染成什么样都无所谓。下游的 MUX 早就把它拉黑了*/
                                    


    //load-use数据冒险除了前递，还需要暂停的触发信号                            
   wire rf_rdata1_hazard = use_rf_rdata1 && (
    exe_valid && exe_is_load && exe_rf_we && (exe_rf_waddr != 0) && (exe_rf_waddr == rf_raddr1) //load-use数据冒险：即使有前递也不行,依然需要暂停。
   );

   wire rf_rdata2_hazard = use_rf_rdata2 && (
    exe_valid && exe_is_load && exe_rf_we && (exe_rf_waddr != 0) && (exe_rf_waddr == rf_raddr2) //同rdata1的冒险分析
   );
    /*  ld.w  r1,  8(r2)
        add.w r3,  r1, r4
        add.w在刚进入exe阶段需要读rdata1,但ld.w最早在mem结束才能将数据前递到exe的输入端，所以需要暂停一个周期
    EXE阶段的指令要有效，需要是ld.w指令，要写回寄存器，并且它写回的寄存器地址不为0，并且它写回的寄存器地址和当前指令读rdata1的寄存器地址相同，那么就有load-use冒险
    */

    
    assign id_ready_go = !(rf_rdata1_hazard || rf_rdata2_hazard);//如果有冒险了，那么id_ready_go就为0，id_stage就不ready，就不允许id_stage的指令进入下一个阶段exe_stage，这样就在id_stage停住了，往后传气泡id_to_exe_valid = id_valid && id_ready_go=0
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


    assign br_cancel = id_valid && br_taken && id_ready_go;
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
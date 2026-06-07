`include "mycpu_top.h"
module wb_stage(
    input  wire                 clk,
    input  wire                 reset,
    
    input  wire [`MEM_TO_WB_BUS_WIDTH-1:0] mem_to_wb_bus,              
    output wire [`WB_TO_ID_BUS_WIDTH-1:0]  wb_to_id_bus,//不要误以为有wb_to_id_bus就需要加wb_t0_id_vaild和id_allow_in了，wb_to_id_bus本来就是写回阶段自己用的，只不过实现模块有一部分在前面的ID阶段，其实还是相当于在wb阶段内部执行
    input wire  mem_to_wb_valid, 
    output wire wb_allow_in, 

    // [新增] WB→CSR 接口（输出给顶层，再连到 csr_regfile）
    output wire        wb_ex,          // 有异常触发（ecall 在本版本）
    output wire [31:0] wb_pc_for_csr,  // 触发异常指令的PC
    output wire [31:0] wb_cause,       // 异常原因（→ mcause）
    output wire [31:0] wb_tval,        // 异常附加信息（→ mtval）
    output wire        mret_flush,     // mret 在 WB 执行，触发流水线冲刷
 
    // CSR 指令写接口（输出给顶层，再连到 csr_regfile）
    output wire        wb_csr_we,      // CSR 写使能
    output wire [11:0] wb_csr_addr,    // CSR 地址
    output wire [31:0] wb_csr_wdata,   // CSR 写入值
 
    // [新增] CSR 读出值（从顶层csr_regfile输入，供CSR指令把旧值写回regfile）
    input  wire [31:0] csr_rvalue,

    output wire [31:0] debug_wb_pc,
    output wire [3:0]  debug_wb_rf_we,
    output wire [4:0]  debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata,
    output wire        debug_wb_valid

);
    
    //pipeline control 
    reg         wb_valid;
    wire        wb_ready_go;

    assign wb_ready_go    = 1'b1;
    assign wb_allow_in    = !wb_valid || wb_ready_go  ; 
    
    always@(posedge clk) begin
        if(reset) begin
            wb_valid <= 1'b0;
        end
        else if(wb_allow_in) begin
            wb_valid <= mem_to_wb_valid;
        end
    end

    //input bus from mem stage
    wire [31:0] wb_pc;
    wire        wb_reg_we;
    wire [4:0]  wb_reg_waddr;
    wire [31:0] wb_final_result;

    // [新增] CSR 透传字段
    wire        wb_inst_ecall;
    wire        wb_inst_mret;
    wire        wb_csr_we_from_bus;
    wire [11:0] wb_csr_addr_from_bus;
    wire [31:0] wb_csr_wdata_from_bus;
    wire        wb_is_csr_inst;

    reg [`MEM_TO_WB_BUS_WIDTH-1:0] wb_reg;

    always @(posedge clk) begin
        if(mem_to_wb_valid && wb_allow_in) begin
            wb_reg <= mem_to_wb_bus;
        end
    end

    assign {
        wb_pc,
        wb_final_result,
        wb_reg_we,
        wb_reg_waddr,
        // [新增]
        wb_inst_ecall,
        wb_inst_mret,
        wb_csr_we_from_bus,
        wb_csr_addr_from_bus,
        wb_csr_wdata_from_bus,
        wb_is_csr_inst
     } = wb_reg;


    //output bus to id stage
    assign wb_to_id_bus = {
        wb_valid,
        wb_reg_we,
        wb_reg_waddr,
        wb_rf_wdata_final   // [修改] CSR指令时写CSR旧值，其他时候保持原来逻辑
    };
    //位宽1+1+5+32=39



     /*..................internal signals................*/


    // ================================================================
    // [新增] 异常检测与触发（任务12：只处理 ecall）
    //
    // 任务12 只需要实现 ecall。当写回级有效且指令是 ecall 时，触发异常：
    //   - wb_ex 置1，通知 IF 级冲刷流水线并跳向 ex_entry（mtvec）
    //   - 同时更新 CSR：mepc←wb_pc，mcause←11，mtval←0，mstatus更新
    // ================================================================
 
    // wb_ex：写回级有异常（本版本仅 ecall）
    assign wb_ex = wb_valid && wb_inst_ecall;
 
    // 触发异常的 PC（写入 mepc）
    assign wb_pc_for_csr = wb_pc;
 
    // 异常原因（写入 mcause）
    // ecall 在 M 模式下：mcause = 11（32'd11）
    assign wb_cause = wb_inst_ecall ? 32'd11 : 32'b0;
 
    // 异常附加信息（写入 mtval）
    // ecall 的 mtval = 0（规范规定 ecall 不提供附加信息）
    assign wb_tval = 32'b0;
 


    // ================================================================
    // [新增] mret 处理
    //
    // mret 在 WB 级执行时：
    //   1. 触发 mret_flush，通知 IF 级冲刷流水线并跳向 csr_mepc_out
    //   2. CSR 模块收到 mret_flush 后自动恢复 mstatus（MIE←MPIE 等）
    //
    // 注意：mret 是普通的流水线控制指令，不触发 wb_ex（不是异常）
    //       除非 mret 本身触发了某种异常（本任务不考虑），wb_ex 和 mret_flush 互斥。
    // ================================================================
    assign mret_flush = wb_valid && wb_inst_mret;
 

    // ================================================================
    // [新增] CSR 指令写操作
    //
    // CSR 写使能：来自 EXE 级计算好的 csr_we（已考虑了 csrrs/csrrc src=0 不写的情况）
    // 注意：异常（wb_ex=1）时优先处理异常，不执行 CSR 指令的写操作
    //       实际上 ecall 不是 CSR 写指令，两者不会同时有效，此处保险起见加上 !wb_ex
    // ================================================================
    assign wb_csr_we    = wb_valid && wb_csr_we_from_bus && !wb_ex;
    assign wb_csr_addr  = wb_csr_addr_from_bus;
    assign wb_csr_wdata = wb_csr_wdata_from_bus;



    // ================================================================
    // 寄存器堆写回
    //
    // 对于 CSR 指令（csrrw/csrrs/csrrc 等）：
    //   - rd 寄存器要写入 CSR 的旧值（在写 CSR 之前读出的值）
    //   - csr_rvalue 是 CSR 模块的异步读出值，本拍就有效
    //   - 所以 CSR 指令的回写数据 = csr_rvalue（在 EXE 级读到的 CSR 旧值）
    //
    // 但等等：EXE 级计算 csr_wdata 时用了 csr_rvalue（此时是旧值），
    //         WB 级 CSR 还没被写入（写入是本拍末尾），所以 WB 级读 csr_rvalue
    //         仍然是旧值 ✓。
    //
    // 因此：CSR 指令把 rd 写成 csr_rvalue（当前 CSR 的旧值）。
    //
    // 注意：ecall/mret 不写通用寄存器（wb_reg_we 来自 ID 级，ecall/mret 的 reg_we=0）
    // ================================================================
    
    // 写回数据选择：
    // - 普通指令：wb_final_result（ALU结果 or 内存读取结果）
    // - CSR指令：csr_rvalue（CSR 的旧值写入 rd）
    //
    // 判断是否是 CSR 指令（需要回写 CSR 旧值）：wb_csr_we_from_bus 为 1
    // 说明这是一条会写 CSR 的指令（csrrw/csrrwi，或 csrrs/csrrc 且 src≠0）
    // 这些指令的 rd 都要写入 CSR 旧值。
    //
    // 对于 csrrs rd, csr, x0（只读CSR，src=0 不写CSR，但 rd 要写 CSR 旧值）：
    // 需要额外 1 位信号来区分。为保持任务12代码简洁，此处暂不处理该边角情况，
    // 实际程序中 csrrs t0, csr, x0 是读 CSR 的标准写法，任务13再完善。
  
    wire [31:0] wb_rf_wdata_final = wb_is_csr_inst ? csr_rvalue : wb_final_result;



    
    // debug info generate
    assign debug_wb_pc       = wb_pc;
    assign debug_wb_rf_we   = {4{wb_reg_we & wb_valid}};//★为什么加上wb_valid？为什么debug_wb_rf_wen是4位的？32位的数据包含了 4个字节。为了方便调试，Trace测试平台要求你的CPU交代得非常详细：你到底写了这32位数据里面的哪几个字节
    assign debug_wb_rf_wnum = wb_reg_waddr;  //写了哪个寄存器(写地址)
    assign debug_wb_rf_wdata = wb_rf_wdata_final;
    assign debug_wb_valid = wb_valid;


/*具体写回的操作已经在ID阶段的寄存器堆那里实现了，这里就不需要再写了，直接把要写回的数据通过总线传回ID阶段就行了

    assign {wb_rf_we, wb_rf_waddr, wb_rf_wdata} = wb_to_id_bus;//输入
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
    //写回
    assign rf_we    = wb_rf_we;//位于WB阶段的前3条指令传回来的写使能信号，用于在ID阶段写回寄存器堆。当前位于ID阶段的指令产生的写使能信号reg_we要传到下一个阶段EXE，等它自己到了WB阶段再用
    assign rf_waddr = wb_rf_waddr;//原理同rf_we
    assign rf_wdata = wb_rf_wdata;//原理同rf_we
*/


endmodule 
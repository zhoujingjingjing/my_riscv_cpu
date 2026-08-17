`include "mycpu_top.h"

// 文件职责：CPU顶层，把七个流水级、存储器、CSR、性能和退休接口连接成完整核心。
// 上游：外部IROM/DRAM、复位和调试输入；内部各stage按程序年龄传递级间总线。
// 下游：外部IROM/DRAM请求、退休调试、性能计数和CSR状态输出。
module mycpu_top #(
    parameter [31:0] RESET_PC = 32'h8000_0000
)(
    input  wire        clk,
    input  wire        resetn,
    // inst sram interface
    output wire        inst_sram_en,
    output wire [3:0]  inst_sram_we,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire [31:0] inst_sram_rdata,
    // [阶段1新增] 指令存储器第二只读口：同拍读取PC+4。
    // B口不提供写信号，正式BRAM的web/dinb会在student_top中固定为0。
    output wire        inst_sram_en_b,
    output wire [31:0] inst_sram_addr_b,
    input  wire [31:0] inst_sram_rdata_b,
    // data sram interface
    output wire        data_sram_en,
    output wire [3:0]  data_sram_we,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    input  wire [31:0] data_sram_rdata,
    // trace debug interface
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,//写使能为什么4位？
    output wire [ 4:0] debug_wb_rf_wnum,//写了哪个寄存器
    output wire [31:0] debug_wb_rf_wdata,//写了什么数据
    output wire        debug_wb_valid,
    // Retirement Interface V2。[阶段4修改] slot0承载较老指令，slot1承载
    // 同组较年轻指令；每拍退休0、1或2条，始终保证从slot0开始连续有效。
    output wire [31:0] retire_if_version,
    output wire [7:0]  retire_count,
    output wire        retire_slot0_valid,
    output wire [31:0] retire_slot0_pc,
    output wire [31:0] retire_slot0_inst,
    output wire [31:0] retire_slot0_dnpc,
    output wire        retire_slot0_rd_we,
    output wire [4:0]  retire_slot0_rd,
    output wire [31:0] retire_slot0_rd_data,
    output wire        retire_slot0_mem_valid,
    output wire        retire_slot0_mem_store,
    output wire [31:0] retire_slot0_mem_addr,
    output wire [3:0]  retire_slot0_mem_mask,
    output wire [31:0] retire_slot0_mem_wdata,
    output wire [31:0] retire_slot0_mem_rdata,
    output wire        retire_slot0_trap_valid,
    output wire        retire_slot0_trap_interrupt,
    output wire [31:0] retire_slot0_trap_cause,
    output wire        retire_slot0_skip_ref,
    output wire        retire_slot1_valid,
    output wire [31:0] retire_slot1_pc,
    output wire [31:0] retire_slot1_inst,
    output wire [31:0] retire_slot1_dnpc,
    output wire        retire_slot1_rd_we,
    output wire [4:0]  retire_slot1_rd,
    output wire [31:0] retire_slot1_rd_data,
    output wire        retire_slot1_mem_valid,
    output wire        retire_slot1_mem_store,
    output wire [31:0] retire_slot1_mem_addr,
    output wire [3:0]  retire_slot1_mem_mask,
    output wire [31:0] retire_slot1_mem_wdata,
    output wire [31:0] retire_slot1_mem_rdata,
    output wire        retire_slot1_trap_valid,
    output wire        retire_slot1_trap_interrupt,
    output wire [31:0] retire_slot1_trap_cause,
    output wire        retire_slot1_skip_ref,
    // 保留V1字段名作为slot0只读别名，避免数字孪生和旧脚本同阶段失效。
    output reg  [31:0] retire_pc,
    output reg  [31:0] retire_inst,
    output reg  [31:0] retire_dnpc,
    output reg         retire_rd_we,
    output reg  [4:0]  retire_rd,
    output reg  [31:0] retire_rd_data,
    output wire [1023:0] retire_gpr_flat,
    output reg         retire_mem_valid,
    output reg         retire_mem_store,
    output reg  [31:0] retire_mem_addr,
    output reg  [3:0]  retire_mem_mask,
    output reg  [31:0] retire_mem_wdata,
    output reg  [31:0] retire_mem_rdata,
    output reg         retire_trap_valid,
    output reg         retire_trap_interrupt,
    output reg  [31:0] retire_trap_cause,
    output reg         retire_skip_ref,

    // [测试框架完善] Performance Interface V1。全部导出原始累计值，
    // 百分比和CPI由测试框架计算，避免RTL中加入除法器。
    output wire [31:0] perf_if_version,
    output wire [63:0] perf_mcycle,
    output wire [63:0] perf_minstret,
    output wire [63:0] perf_load_use_stall_count,
    output wire [63:0] perf_branch_count,
    output wire [63:0] perf_mispredict_count,
    // [阶段2新增] 影子配对累计值，仅供验证和后续架构决策。
    output wire [63:0] perf_pair_count,
    output wire [63:0] perf_pair_ok_count,
    output wire [63:0] perf_pair_raw_count,
    output wire [63:0] perf_pair_waw_count,
    output wire [63:0] perf_pair_lsu_count,
    output wire [63:0] perf_pair_mul_count,
    output wire [63:0] perf_pair_ctrl_count,
    output wire [63:0] perf_pair_serial_count
);

    //============================================================
    // reset synchronizer
    //============================================================
    // 外部resetn为低有效；内部各级沿用单发射时期的高有效reset命名。
    reg reset;
    always @(posedge clk) reset <= ~resetn;

/*.............信号定义..........................*/

    //============================================================
    // seven-stage pipeline buses
    //============================================================
    // 这里按IF1→IF2→ID→ISSUE→EXE→MEM→WB顺序集中列线，便于顺着流水方向检查连接。
    // 白话信号导读：
    // *_valid表示“这一站送来的包裹是真的”，*_allow_in表示“下一站现在能不能收包裹”。
    // *_bus是级间包裹本身，里面按mycpu_top.h约定塞着PC、指令、预测、操作数、控制位等字段。
    // 带后缀1的信号都是slot1，也就是同一拍里较年轻的第二条指令；不带1的是slot0，年龄更老。
    // bypass_bus不是普通流水包裹，而是给ISSUE看的“前方施工牌”：谁会写rd、结果在哪、是不是还要等load/MUL。
    // flush相关信号像“全线改道通知”，来自EXE分支纠错或WB异常/mret，优先级高于普通valid传递。

    // IF1只负责地址和预测；IF2保存同步IROM请求并维护取指队列。
    // if_to_if2_valid/if_to_if2_bus：IF1发给IF2的一次取指请求，主要是取指PC和预测快照。
    // if2_allow_in：IF2告诉IF1“我的取指队列还有空位，可以继续发下一个PC”。
    // if_ras_ptr：前端返回地址栈当前指针，ID在识别call/ret时用它维护预测栈。
    // if_flush：前端自己记录的flush状态，用来避免冲刷期间继续把旧路径指令送下去。
    wire if_to_if2_valid;
    wire [`IF_TO_IF2_BUS_WIDTH-1:0] if_to_if2_bus;
    wire if2_allow_in;
    wire [2:0] if_ras_ptr;
    wire if_flush;

    // if_to_id_valid/if_to_id_bus：IF2送给ID的slot0指令包，里面有PC、指令和分支预测信息。
    // if_to_id_valid1/if_to_id_bus1：同一拍的slot1候选指令包，ID会决定它能不能和slot0一起发射。
    wire if_to_id_valid;
    wire [`IF_TO_ID_BUS_WIDTH-1:0] if_to_id_bus;
    // [阶段2新增、阶段4接通] FIFO第二槽先用于影子译码，现已进入真实双发通路。
    wire if_to_id_valid1;
    wire [`FETCH_BUS_WIDTH-1:0] if_to_id_bus1;
    // 两条随 FIFO 出队的轻量预译码标签，只供 ID 做双发射配对使用。
    wire [24:0] if_to_id_dec0;
    wire [24:0] if_to_id_dec1;

    // id_to_if_bus：ID维护RAS时给IF的push/pop/writeback信息，不是普通指令流。
    // id_to_issue_valid/id_to_issue_bus：ID译码后的slot0资料，交给ISSUE读寄存器和做冒险判断。
    // id_to_issue_valid1/id_to_issue_bus1：ID译码后的slot1资料，只有配对成功时才作为第二条继续向前。
    // id_to_issue_rf_bus：两条指令的四个源寄存器地址打成一包，专门送regfile四个读口。
    // id_allow_in/issue_allow_in：ID和ISSUE之间的握手，issue堵住时ID必须保持当前译码结果。
    // issue_to_exe_valid*/issue_to_exe_bus*：ISSUE已经把操作数读好、前递修好后送给EXE的执行包。
    // id_take2：告诉IF2这拍ID真的吃掉了两条指令；如果为0，第二条还要留在取指队列里下次再来。
    // exe_to_if_bus：EXE算出的真实分支/跳转结果，预测错时带着正确PC回前端修路。
    wire [`ID_TO_IF_BUS_WIDTH-1:0] id_to_if_bus;

    wire id_to_issue_valid;
    wire id_to_issue_valid1;
    wire id_allow_in;
    wire issue_allow_in;
    wire [`ID_TO_ISSUE_BUS_WIDTH-1:0] id_to_issue_bus;
    wire [`ID_TO_ISSUE_BUS_WIDTH-1:0] id_to_issue_bus1;
    wire [`ID_TO_ISSUE_RF_BUS_WIDTH-1:0] id_to_issue_rf_bus;
    wire issue_to_exe_valid;
    wire issue_to_exe_valid1;
    wire [`ISSUE_TO_EXE_BUS_WIDTH-1:0] issue_to_exe_bus;
    wire [`ISSUE_TO_EXE_BUS_WIDTH-1:0] issue_to_exe_bus1;
    wire id_take2;
    wire [`EXE_TO_IF_BUS_WIDTH-1:0] exe_to_if_bus;

    // exe_to_mem_valid*/exe_to_mem_bus*：EXE送给MEM的两个结果包，含ALU/地址/CSR/异常/后置ALU材料。
    // exe_allow_in/mem_allow_in：EXE和MEM之间的握手，MEM被DRAM或后置计算拖住时EXE也要停住。
    // exe_to_id_bypass_bus*：EXE给ISSUE的前递/冒险牌，说明本槽rd、结果、load、late和flush情况。
    // mul_valid/mul_result：共享乘法流水线在EXE/MEM边界旁边给出的结果，与对应MUL槽同拍进入MEM。
    // exe_flush：两个EXE旁路包里只要有一个说“预测错了”，ID/ISSUE就立即清掉旧路径指令。
    wire exe_to_mem_valid;
    wire exe_to_mem_valid1;
    wire exe_allow_in;
    wire [`EXE_TO_MEM_BUS_WIDTH-1:0] exe_to_mem_bus;
    wire [`EXE_TO_MEM_BUS_WIDTH-1:0] exe_to_mem_bus1;
    wire [`EXE_TO_ID_BYPASS_BUS_WIDTH-1:0] exe_to_id_bypass_bus;
    wire [`EXE_TO_ID_BYPASS_BUS_WIDTH-1:0] exe_to_id_bypass_bus1;
    // [阶段5A新增] 共享MUL结果与进入MEM的MUL槽严格同拍。
    wire mul_valid;
    wire [31:0] mul_result;
    // [阶段3新增] EXE纠错信号也直接冲刷ID和ISSUE，不串过额外译码逻辑。
    wire exe_flush = exe_to_id_bypass_bus[0] || exe_to_id_bypass_bus1[0];

    // mem_to_wb_valid_raw*是MEM本槽原始完成脉冲；mem_to_wb_valid*是考虑顺序和flush后的真正WB入口valid。
    // mem_to_wb_bus*：MEM整理后的最终写回包，WB只负责按年龄提交和导出退休信息。
    // mem_to_id_bypass_bus*：MEM给ISSUE的前递牌，load/MUL/后置ALU这种晚结果主要靠它解除等待。
    // wb_allow_in在当前设计恒为1，但仍保留这根线，让流水级接口形式和前面几级保持一致。
    // wb_to_id_bus*：WB写回信息回送ISSUE，解决最靠后的前递，也给regfile双写口使用。
    wire mem_to_wb_valid;
    wire mem_to_wb_valid_raw;
    wire mem_to_wb_valid1;
    wire mem_to_wb_valid1_raw;
    wire mem_allow_in;
    wire [`MEM_TO_WB_BUS_WIDTH-1:0] mem_to_wb_bus;
    wire [`MEM_TO_WB_BUS_WIDTH-1:0] mem_to_wb_bus1;
    wire [`MEM_TO_ID_BYPASS_BUS_WIDTH-1:0] mem_to_id_bypass_bus;
    wire [`MEM_TO_ID_BYPASS_BUS_WIDTH-1:0] mem_to_id_bypass_bus1;

    wire wb_allow_in;
    wire [`WB_TO_ID_BUS_WIDTH-1:0] wb_to_id_bus;
    wire [`WB_TO_ID_BUS_WIDTH-1:0] wb_to_id_bus1;


    //============================================================
    // CSR, debug and performance wires
    //============================================================
    // CSR写入只从WB提交，EXE只有只读口；debug/retire字段从WB再打一拍供NEMU比对。
    // ---- [新增] WB→CSR 接口信号 ----
    // wb_ex是WB确认的精确异常，只在最老的可提交指令上产生；wb_pc_for_csr会写进mepc。
    // wb_cause/wb_tval是异常原因和附加信息，目前ecall主要给cause=11，tval保留为0。
    // mret_flush表示mret提交成功，前端要从csr_mepc_out重新取指。
    // wb_csr_we/wb_csr_addr/wb_csr_wdata是WB向CSR寄存器堆提交的写端口，保证软件可见状态按程序顺序更新。
    wire        wb_ex;          // WB级有异常触发
    wire [31:0] wb_pc_for_csr; // WB级PC（供CSR保存mepc）
    wire [31:0] wb_cause;       // 异常原因
    wire [31:0] wb_tval;        // 异常附加信息
    wire        mret_flush;     // mret在WB级执行
    wire        wb_csr_we;      // WB级CSR写使能
    wire [11:0] wb_csr_addr;   // WB级CSR地址
    wire [31:0] wb_csr_wdata;  // WB级CSR写入值

    // ---- [新增] CSR→流水线 接口信号 ----
    // ex_entry是异常入口地址，csr_mepc_out是mret返回地址，二者都会送回IF选择下一条PC。
    // exe_csr_raddr/exe_csr_rvalue给EXE算CSR新值用；wb_csr_rvalue给WB写回rd的“CSR旧值”用。
    // debug_gpr_flat是regfile提供的32个通用寄存器平面图，退休接口用它和NEMU比对整机状态。
    wire [31:0] ex_entry;       // 异常入口（→IF级nextpc）
    wire [31:0] csr_mepc_out;   // mepc（→IF级nextpc，供mret用）
    // [第4步验证修复] CSR提供EXE/WB两个独立异步读口。
    wire [11:0] exe_csr_raddr;
    wire [31:0] exe_csr_rvalue;
    wire [31:0] wb_csr_rvalue;
    wire [1023:0] debug_gpr_flat;
    // debug_wb_*是slot0刚到WB时的退休元数据；下一拍锁进retire_*，与已经写好的GPR快照对齐。
    // debug_wb_mem_*只在访存指令有效时给NEMU检查地址、mask、写数据和读数据。
    // debug_wb_trap_*把异常退休事件带给测试框架，普通指令这些字段保持无效即可。
    wire [31:0] debug_wb_inst;
    wire [31:0] debug_wb_dnpc;
    wire debug_wb_mem_valid;
    wire debug_wb_mem_store;
    wire [31:0] debug_wb_mem_addr;
    wire [3:0] debug_wb_mem_mask;
    wire [31:0] debug_wb_mem_wdata;
    wire [31:0] debug_wb_mem_rdata;
    wire debug_wb_trap_valid;
    wire [31:0] debug_wb_trap_cause;
    // [阶段4新增] slot1写回/退休元数据，字段与slot0完全对称。
    // debug_wb_*1是slot1的同款退休元数据；它只描述年轻普通指令，不负责CSR、异常和mret。
    wire [31:0] debug_wb_pc1;
    wire [3:0] debug_wb_rf_we1;
    wire [4:0] debug_wb_rf_wnum1;
    wire [31:0] debug_wb_rf_wdata1;
    wire debug_wb_valid1;
    wire [31:0] debug_wb_inst1;
    wire [31:0] debug_wb_dnpc1;
    wire debug_wb_mem_valid1;
    wire debug_wb_mem_store1;
    wire [31:0] debug_wb_mem_addr1;
    wire [3:0] debug_wb_mem_mask1;
    wire [31:0] debug_wb_mem_wdata1;
    wire [31:0] debug_wb_mem_rdata1;
    wire debug_wb_trap_valid1;
    wire [31:0] debug_wb_trap_cause1;
    // [测试框架完善] 从ID/EXE级导出的单周期性能事件。
    // perf_load_use_stall_event：ISSUE因为load-use或晚结果等待而停住的单拍事件。
    // perf_branch_event/perf_branch_mispredict_event：EXE确认的分支数量和预测失败数量。
    // perf_pair_*_event：ID尝试双发、成功双发以及RAW/WAW/LSU/MUL/控制流/串行指令阻塞原因统计。
    wire perf_load_use_stall_event;
    wire perf_branch_event;
    wire perf_branch_mispredict_event;
    // [阶段2新增] ID影子配对的单周期事件。
    wire perf_pair_event;
    wire perf_pair_ok_event;
    wire perf_pair_raw_event;
    wire perf_pair_waw_event;
    wire perf_pair_lsu_event;
    wire perf_pair_mul_event;
    wire perf_pair_ctrl_event;
    wire perf_pair_serial_event;



/*.............模块实例化..........................*/
  //============================================================
  // IF1/IF2/ID/ISSUE/EXE/MEM/WB instances
  //============================================================
  // 实例顺序严格按流水级排列。旁路和flush信号在顶层只连线，不在这里重新组合复杂逻辑。
  if_stage #(
      .RESET_PC(RESET_PC)
  ) if_stage (
      .clk(clk),
      .reset(reset),
      .if_to_if2_valid(if_to_if2_valid),
      .if_to_if2_bus(if_to_if2_bus),
      .if2_allow_in(if2_allow_in),
      .if_ras_ptr(if_ras_ptr),
      .if_flush(if_flush),
      // [新增] 接收来自 ID 的 RAS 维护信号 (push/pop/wdata)
      .id_to_if_bus(id_to_if_bus),
      .exe_to_if_bus(exe_to_if_bus),
      // [新增] 异常/mret时的流水线冲刷和跳转目标
      .wb_ex(wb_ex),
      .ex_entry(ex_entry),
      .mret_flush(mret_flush),
      .csr_mepc_out(csr_mepc_out),

      .inst_sram_en(inst_sram_en),
      .inst_sram_we(inst_sram_we),
      .inst_sram_addr(inst_sram_addr),
      .inst_sram_wdata(inst_sram_wdata),
      // [阶段1新增] 把IF发出的PC+4请求直接接到CPU顶层B口。
      .inst_sram_en_b(inst_sram_en_b),
      .inst_sram_addr_b(inst_sram_addr_b)
    );

  // 第二取指级接收同步IROM返回，并继续使用原有IF到ID总线名称。
  if2_stage if2_stage (
      .clk(clk),
      .reset(reset),
      .if_to_if2_valid(if_to_if2_valid),
      .if_to_if2_bus(if_to_if2_bus),
      .if2_allow_in(if2_allow_in),
      .if_ras_ptr(if_ras_ptr),
      .if_flush(if_flush),
      .inst_sram_rdata(inst_sram_rdata),
      .inst_sram_rdata_b(inst_sram_rdata_b),
      .if_to_id_bus(if_to_id_bus),
      .if_to_id_bus1(if_to_id_bus1),
      .if_to_id_dec0(if_to_id_dec0),
      .if_to_id_dec1(if_to_id_dec1),
      .if_to_id_valid(if_to_id_valid),
      .if_to_id_valid1(if_to_id_valid1),
      .id_allow_in(id_allow_in),
      .id_take2(id_take2)
    );

  id_stage  id_stage (
      .clk(clk),
      .reset(reset),
      .id_to_issue_bus(id_to_issue_bus),
      .id_to_issue_bus1(id_to_issue_bus1),
      .id_to_issue_rf_bus(id_to_issue_rf_bus),
      // [新增] 输出给 IF 的 RAS 维护信号
      .id_to_if_bus(id_to_if_bus),
      .if_to_id_bus(if_to_id_bus),
      .if_to_id_bus1(if_to_id_bus1),
      .if_to_id_dec0(if_to_id_dec0),
      .if_to_id_dec1(if_to_id_dec1),
      .id_allow_in(id_allow_in),
      .id_take2(id_take2),
      .id_to_issue_valid(id_to_issue_valid),
      .id_to_issue_valid1(id_to_issue_valid1),
      .issue_allow_in(issue_allow_in),
      .if_to_id_valid(if_to_id_valid),
      .if_to_id_valid1(if_to_id_valid1),
      // [新增] CSR冲突阻塞：WB级告知ID级有CSR写指令或mret在执行
      .wb_ex(wb_ex),
      .mret_flush(mret_flush),
      .flush_en(exe_flush),
      .perf_pair(perf_pair_event),
      .perf_pair_ok(perf_pair_ok_event),
      .perf_pair_raw(perf_pair_raw_event),
      .perf_pair_waw(perf_pair_waw_event),
      .perf_pair_lsu(perf_pair_lsu_event),
      .perf_pair_mul(perf_pair_mul_event),
      .perf_pair_ctrl(perf_pair_ctrl_event),
      .perf_pair_serial(perf_pair_serial_event)
    );

  // [阶段3新增、阶段4扩展] 正式双槽ISSUE流水级。
  issue_stage issue_stage (
      .clk(clk),
      .reset(reset),
      .id_to_issue_bus(id_to_issue_bus),
      .id_to_issue_bus1(id_to_issue_bus1),
      .id_to_issue_rf_bus(id_to_issue_rf_bus),
      .id_to_issue_valid(id_to_issue_valid),
      .id_to_issue_valid1(id_to_issue_valid1),
      .id_allow_in(issue_allow_in),
      .issue_to_exe_bus(issue_to_exe_bus),
      .issue_to_exe_bus1(issue_to_exe_bus1),
      .issue_to_exe_valid(issue_to_exe_valid),
      .issue_to_exe_valid1(issue_to_exe_valid1),
      .exe_allow_in(exe_allow_in),
      .exe_to_issue_bus(exe_to_id_bypass_bus),
      .exe_to_issue_bus1(exe_to_id_bypass_bus1),
      .mem_to_issue_bus(mem_to_id_bypass_bus),
      .mem_to_issue_bus1(mem_to_id_bypass_bus1),
      .wb_to_issue_bus(wb_to_id_bus),
      .wb_to_issue_bus1(wb_to_id_bus1),
      .exe_flush(exe_flush),
      .wb_ex(wb_ex),
      .mret_flush(mret_flush),
      .debug_gpr_flat(debug_gpr_flat),
      .perf_load_use_stall(perf_load_use_stall_event)
    );

  exe_stage  exe_stage (
      .clk(clk),
      .reset(reset),
      .id_to_exe_bus(issue_to_exe_bus),
      .id_to_exe_bus1(issue_to_exe_bus1),
      .exe_to_mem_bus(exe_to_mem_bus),
      .exe_to_mem_bus1(exe_to_mem_bus1),
      .exe_to_if_bus(exe_to_if_bus),
      .exe_to_id_bypass_bus(exe_to_id_bypass_bus),
      .exe_to_id_bypass_bus1(exe_to_id_bypass_bus1),
      .id_to_exe_valid(issue_to_exe_valid),
      .id_to_exe_valid1(issue_to_exe_valid1),
      .mem_allow_in(mem_allow_in),
      .exe_allow_in(exe_allow_in),
      .exe_to_mem_valid(exe_to_mem_valid),
      .exe_to_mem_valid1(exe_to_mem_valid1),
      .mul_valid_o(mul_valid),
      .mul_result(mul_result),
      // [RV32M移植] 取消被更老异常/mret冲刷的年轻M指令。
      .wb_ex(wb_ex),
      .mret_flush(mret_flush),
      // [第4步验证修复] EXE专用CSR读口。
      .csr_raddr(exe_csr_raddr),
      .csr_rvalue(exe_csr_rvalue),
      // [阶段5C2新增] LW同步读返回值直接进入EXE局部修复旁路。
      .data_sram_rdata(data_sram_rdata),
      .data_sram_en(data_sram_en),
      .data_sram_we(data_sram_we),
      .data_sram_addr(data_sram_addr),
      .data_sram_wdata(data_sram_wdata),
      .perf_branch(perf_branch_event),
      .perf_branch_mispredict(perf_branch_mispredict_event)
    );

  mem_stage  mem_stage (
    .clk(clk),
    .reset(reset),
    .exe_to_mem_bus(exe_to_mem_bus),
    .exe_to_mem_bus1(exe_to_mem_bus1),
    .mem_to_wb_bus(mem_to_wb_bus),
    .mem_to_wb_bus1(mem_to_wb_bus1),
    .mem_to_id_bypass_bus(mem_to_id_bypass_bus),
    .mem_to_id_bypass_bus1(mem_to_id_bypass_bus1),
    .exe_to_mem_valid(exe_to_mem_valid),
    .exe_to_mem_valid1(exe_to_mem_valid1),
    .wb_allow_in(wb_allow_in),
    .mem_allow_in(mem_allow_in),
    .mem_to_wb_valid(mem_to_wb_valid_raw),
    .mem_to_wb_valid1(mem_to_wb_valid1_raw),
    .data_sram_rdata(data_sram_rdata),
    .mul_valid(mul_valid),
    .mul_result(mul_result)
  );

  // [RV32M移植] ecall/mret在WB确定冲刷时，MEM里仍可能有一条更年轻指令。
  // 必须阻止它进入WB，否则EXE里的M指令即使已cancel，前一条年轻指令仍会错误退休。
  // 这里只过滤valid，不改原MEM→WB数据总线，也不创建新的退休通路。
  assign mem_to_wb_valid = mem_to_wb_valid_raw && !wb_ex && !mret_flush;
  assign mem_to_wb_valid1 = mem_to_wb_valid1_raw && !wb_ex && !mret_flush;

  wb_stage  wb_stage (
    .clk(clk),
    .reset(reset),
    .mem_to_wb_bus(mem_to_wb_bus),
    .mem_to_wb_bus1(mem_to_wb_bus1),
    .wb_to_id_bus(wb_to_id_bus),
    .wb_to_id_bus1(wb_to_id_bus1),
    .mem_to_wb_valid(mem_to_wb_valid),
    .mem_to_wb_valid1(mem_to_wb_valid1),
    .wb_allow_in(wb_allow_in),
    // [新增] WB→CSR 接口
    .wb_ex(wb_ex),
    .wb_pc_for_csr(wb_pc_for_csr),
    .wb_cause(wb_cause),
    .wb_tval(wb_tval),
    .mret_flush(mret_flush),
    .wb_csr_we(wb_csr_we),
    .wb_csr_addr(wb_csr_addr),
    .wb_csr_wdata(wb_csr_wdata),
    // [新增] CSR读值（供WB级CSR指令返回值写回regfile）
    .csr_rvalue(wb_csr_rvalue),
    .debug_wb_pc(debug_wb_pc),
    .debug_wb_rf_we(debug_wb_rf_we),
    .debug_wb_rf_wnum(debug_wb_rf_wnum),
    .debug_wb_rf_wdata(debug_wb_rf_wdata),
    .debug_wb_valid(debug_wb_valid),
    .debug_wb_inst(debug_wb_inst),
    .debug_wb_dnpc(debug_wb_dnpc),
    .debug_wb_mem_valid(debug_wb_mem_valid),
    .debug_wb_mem_store(debug_wb_mem_store),
    .debug_wb_mem_addr(debug_wb_mem_addr),
    .debug_wb_mem_mask(debug_wb_mem_mask),
    .debug_wb_mem_wdata(debug_wb_mem_wdata),
    .debug_wb_mem_rdata(debug_wb_mem_rdata),
    .debug_wb_trap_valid(debug_wb_trap_valid),
    .debug_wb_trap_cause(debug_wb_trap_cause)
    ,.debug_wb_pc1(debug_wb_pc1)
    ,.debug_wb_rf_we1(debug_wb_rf_we1)
    ,.debug_wb_rf_wnum1(debug_wb_rf_wnum1)
    ,.debug_wb_rf_wdata1(debug_wb_rf_wdata1)
    ,.debug_wb_valid1(debug_wb_valid1)
    ,.debug_wb_inst1(debug_wb_inst1)
    ,.debug_wb_dnpc1(debug_wb_dnpc1)
    ,.debug_wb_mem_valid1(debug_wb_mem_valid1)
    ,.debug_wb_mem_store1(debug_wb_mem_store1)
    ,.debug_wb_mem_addr1(debug_wb_mem_addr1)
    ,.debug_wb_mem_mask1(debug_wb_mem_mask1)
    ,.debug_wb_mem_wdata1(debug_wb_mem_wdata1)
    ,.debug_wb_mem_rdata1(debug_wb_mem_rdata1)
    ,.debug_wb_trap_valid1(debug_wb_trap_valid1)
    ,.debug_wb_trap_cause1(debug_wb_trap_cause1)
  );

  //============================================================
  // CSR register file and performance counters
  //============================================================
  // ---- [新增] CSR寄存器堆 ----
  csr_regfile  u_csr_regfile (
    .clk          (clk           ),
    .reset        (reset         ),
    // 指令访问接口（WB级）
    .csr_addr     (wb_csr_addr   ),
    .csr_we       (wb_csr_we     ),
    .csr_wvalue   (wb_csr_wdata  ),
    .csr_rvalue   (wb_csr_rvalue ),
    // [第4步验证修复] EXE读口只读，不参与WB写地址选择。
    .exe_csr_addr (exe_csr_raddr ),
    .exe_csr_rvalue(exe_csr_rvalue),
    // 异常触发接口
    .wb_ex        (wb_ex         ),
    .wb_pc        (wb_pc_for_csr ),
    .wb_cause     (wb_cause      ),
    .wb_tval      (wb_tval       ),
    // mret接口
    .mret_flush   (mret_flush    ),
    // [测试框架完善] WB有效表示本拍真正退休；EXE事件还要排除同拍
    // 被更老异常/mret冲刷的年轻指令。
    .retire_count ({7'b0, debug_wb_valid} +
                   {7'b0, debug_wb_valid1}),
    .load_use_stall_event(perf_load_use_stall_event),
    .branch_event(perf_branch_event && !wb_ex && !mret_flush),
    .branch_mispredict_event(perf_branch_mispredict_event && !wb_ex && !mret_flush),
    .pair_event(perf_pair_event),
    .pair_ok_event(perf_pair_ok_event),
    .pair_raw_event(perf_pair_raw_event),
    .pair_waw_event(perf_pair_waw_event),
    .pair_lsu_event(perf_pair_lsu_event),
    .pair_mul_event(perf_pair_mul_event),
    .pair_ctrl_event(perf_pair_ctrl_event),
    .pair_serial_event(perf_pair_serial_event),
    // 输出到流水线
    .ex_entry     (ex_entry      ),
    .csr_mepc_out (csr_mepc_out  ),
    .perf_mcycle(perf_mcycle),
    .perf_minstret(perf_minstret),
    .perf_load_use_stall_count(perf_load_use_stall_count),
    .perf_branch_count(perf_branch_count),
    .perf_mispredict_count(perf_mispredict_count),
    .perf_pair_count(perf_pair_count),
    .perf_pair_ok_count(perf_pair_ok_count),
    .perf_pair_raw_count(perf_pair_raw_count),
    .perf_pair_waw_count(perf_pair_waw_count),
    .perf_pair_lsu_count(perf_pair_lsu_count),
    .perf_pair_mul_count(perf_pair_mul_count),
    .perf_pair_ctrl_count(perf_pair_ctrl_count),
    .perf_pair_serial_count(perf_pair_serial_count),
    // Hart ID
    .coreid_in    (32'b0         )   // 单核，Hart ID = 0
  );

  //============================================================
  // Retirement Interface V2
  //============================================================
  // 对外退休接口比旧debug口晚一拍：这时寄存器堆写入已经生效，
  // retire_gpr_flat与slot0/slot1退休元数据属于同一个架构状态点。
  // retire_valid/retire_valid1是最终对外退休有效位，已经比debug_wb_valid晚一拍。
  // retire_count直接由两个valid相加得到，只可能是0、1、2；不会出现slot0无效但slot1有效。
  assign retire_if_version = 32'd2;
  // [测试框架完善] 性能接口版本与退休接口分开演进。
  assign perf_if_version = 32'd1;
  assign retire_slot0_valid = retire_valid;
  assign retire_slot1_valid = retire_valid1;
  // [阶段4修改] 两槽紧凑退休：slot1有效时slot0必定有效，计数只能为0/1/2。
  assign retire_count = {7'b0, retire_slot0_valid} +
                        {7'b0, retire_slot1_valid};
  assign retire_gpr_flat = debug_gpr_flat;
  reg retire_valid;
  reg retire_valid1;

  // slot0继续复用已经通过NEMU的V1退休元数据，兼容旧调试接口。
  assign retire_slot0_pc = retire_pc;
  assign retire_slot0_inst = retire_inst;
  assign retire_slot0_dnpc = retire_dnpc;
  assign retire_slot0_rd_we = retire_rd_we;
  assign retire_slot0_rd = retire_rd;
  assign retire_slot0_rd_data = retire_rd_data;
  assign retire_slot0_mem_valid = retire_mem_valid;
  assign retire_slot0_mem_store = retire_mem_store;
  assign retire_slot0_mem_addr = retire_mem_addr;
  assign retire_slot0_mem_mask = retire_mem_mask;
  assign retire_slot0_mem_wdata = retire_mem_wdata;
  assign retire_slot0_mem_rdata = retire_mem_rdata;
  assign retire_slot0_trap_valid = retire_trap_valid;
  assign retire_slot0_trap_interrupt = retire_trap_interrupt;
  assign retire_slot0_trap_cause = retire_trap_cause;
  assign retire_slot0_skip_ref = retire_skip_ref;

  // [阶段4新增] slot1退休寄存器。元数据与同一时钟沿完成的第二写口
  // 一起延迟一拍，使NEMU看到的两槽GPR快照严格对应slot0后、slot1后的顺序。
  // retire_pc1/retire_inst1/retire_dnpc1记录第二条退休指令从哪来、是什么、下一条该去哪。
  // retire_rd_we1/retire_rd1/retire_rd_data1记录slot1是否写rd、写哪个rd、写入什么值。
  // retire_mem_*1记录slot1访存痕迹；基础双发通常不让复杂控制状态进slot1，但普通load/store仍要对拍。
  // retire_trap_*1和retire_skip_ref1保留对称接口，当前slot1正常不会产生精确异常。
  reg [31:0] retire_pc1;
  reg [31:0] retire_inst1;
  reg [31:0] retire_dnpc1;
  reg retire_rd_we1;
  reg [4:0] retire_rd1;
  reg [31:0] retire_rd_data1;
  reg retire_mem_valid1;
  reg retire_mem_store1;
  reg [31:0] retire_mem_addr1;
  reg [3:0] retire_mem_mask1;
  reg [31:0] retire_mem_wdata1;
  reg [31:0] retire_mem_rdata1;
  reg retire_trap_valid1;
  reg retire_trap_interrupt1;
  reg [31:0] retire_trap_cause1;
  reg retire_skip_ref1;

  assign retire_slot1_pc = retire_pc1;
  assign retire_slot1_inst = retire_inst1;
  assign retire_slot1_dnpc = retire_dnpc1;
  assign retire_slot1_rd_we = retire_rd_we1;
  assign retire_slot1_rd = retire_rd1;
  assign retire_slot1_rd_data = retire_rd_data1;
  assign retire_slot1_mem_valid = retire_mem_valid1;
  assign retire_slot1_mem_store = retire_mem_store1;
  assign retire_slot1_mem_addr = retire_mem_addr1;
  assign retire_slot1_mem_mask = retire_mem_mask1;
  assign retire_slot1_mem_wdata = retire_mem_wdata1;
  assign retire_slot1_mem_rdata = retire_mem_rdata1;
  assign retire_slot1_trap_valid = retire_trap_valid1;
  assign retire_slot1_trap_interrupt = retire_trap_interrupt1;
  assign retire_slot1_trap_cause = retire_trap_cause1;
  assign retire_slot1_skip_ref = retire_skip_ref1;

  // 在这个CPU中，寄存器堆写入发生在WB信息出现后的时钟沿。
  // 所以把WB元数据寄存一拍；时钟沿后，元数据与已经更新的GPR快照属于同一条指令。
  always @(posedge clk) begin
    if (reset) begin
      retire_valid <= 1'b0;
      retire_valid1 <= 1'b0;
      retire_pc <= 32'b0;
      retire_inst <= 32'b0;
      retire_dnpc <= 32'b0;
      retire_rd_we <= 1'b0;
      retire_rd <= 5'b0;
      retire_rd_data <= 32'b0;
      retire_mem_valid <= 1'b0;
      retire_mem_store <= 1'b0;
      retire_mem_addr <= 32'b0;
      retire_mem_mask <= 4'b0;
      retire_mem_wdata <= 32'b0;
      retire_mem_rdata <= 32'b0;
      retire_trap_valid <= 1'b0;
      retire_trap_interrupt <= 1'b0;
      retire_trap_cause <= 32'b0;
      retire_skip_ref <= 1'b0;
      retire_pc1 <= 32'b0;
      retire_inst1 <= 32'b0;
      retire_dnpc1 <= 32'b0;
      retire_rd_we1 <= 1'b0;
      retire_rd1 <= 5'b0;
      retire_rd_data1 <= 32'b0;
      retire_mem_valid1 <= 1'b0;
      retire_mem_store1 <= 1'b0;
      retire_mem_addr1 <= 32'b0;
      retire_mem_mask1 <= 4'b0;
      retire_mem_wdata1 <= 32'b0;
      retire_mem_rdata1 <= 32'b0;
      retire_trap_valid1 <= 1'b0;
      retire_trap_interrupt1 <= 1'b0;
      retire_trap_cause1 <= 32'b0;
      retire_skip_ref1 <= 1'b0;
    end else begin
      retire_valid <= debug_wb_valid;
      retire_valid1 <= debug_wb_valid1;
      retire_pc <= debug_wb_pc;
      retire_inst <= debug_wb_inst;
      retire_dnpc <= debug_wb_trap_valid ? ex_entry :
                     mret_flush ? csr_mepc_out : debug_wb_dnpc;
      retire_rd_we <= (|debug_wb_rf_we) && (debug_wb_rf_wnum != 5'b0);
      retire_rd <= debug_wb_rf_wnum;
      retire_rd_data <= debug_wb_rf_wdata;
      retire_mem_valid <= debug_wb_mem_valid;
      retire_mem_store <= debug_wb_mem_store;
      retire_mem_addr <= debug_wb_mem_addr;
      retire_mem_mask <= debug_wb_mem_mask;
      retire_mem_wdata <= debug_wb_mem_wdata;
      retire_mem_rdata <= debug_wb_mem_rdata;
      retire_trap_valid <= debug_wb_trap_valid;
      retire_trap_interrupt <= debug_wb_trap_cause[31];
      retire_trap_cause <= debug_wb_trap_cause;
      retire_skip_ref <= debug_wb_mem_valid &&
                         (debug_wb_mem_addr >= 32'h8020_0000);
      retire_pc1 <= debug_wb_pc1;
      retire_inst1 <= debug_wb_inst1;
      retire_dnpc1 <= debug_wb_dnpc1;
      retire_rd_we1 <= (|debug_wb_rf_we1) && (debug_wb_rf_wnum1 != 5'b0);
      retire_rd1 <= debug_wb_rf_wnum1;
      retire_rd_data1 <= debug_wb_rf_wdata1;
      retire_mem_valid1 <= debug_wb_mem_valid1;
      retire_mem_store1 <= debug_wb_mem_store1;
      retire_mem_addr1 <= debug_wb_mem_addr1;
      retire_mem_mask1 <= debug_wb_mem_mask1;
      retire_mem_wdata1 <= debug_wb_mem_wdata1;
      retire_mem_rdata1 <= debug_wb_mem_rdata1;
      retire_trap_valid1 <= debug_wb_trap_valid1;
      retire_trap_interrupt1 <= debug_wb_trap_cause1[31];
      retire_trap_cause1 <= debug_wb_trap_cause1;
      retire_skip_ref1 <= debug_wb_mem_valid1 &&
                          (debug_wb_mem_addr1 >= 32'h8020_0000);
    end
  end

endmodule

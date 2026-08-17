`include "mycpu_top.h"

/*
文件职责：ISSUE 读取两条指令的源寄存器，并处理前递、load-use 停顿和晚结果等待。
上游输入来自 ID、EXE、MEM、WB 和寄存器堆；输出送给 EXE、寄存器堆读口和调试接口。
*/
module issue_stage(
    input  wire clk,
    input  wire reset,

    input  wire [`ID_TO_ISSUE_BUS_WIDTH-1:0] id_to_issue_bus,
    input  wire [`ID_TO_ISSUE_BUS_WIDTH-1:0] id_to_issue_bus1,
    input  wire [`ID_TO_ISSUE_RF_BUS_WIDTH-1:0] id_to_issue_rf_bus,
    input  wire id_to_issue_valid,
    input  wire id_to_issue_valid1,
    output wire id_allow_in,

    output wire [`ISSUE_TO_EXE_BUS_WIDTH-1:0] issue_to_exe_bus,
    output wire [`ISSUE_TO_EXE_BUS_WIDTH-1:0] issue_to_exe_bus1,
    output wire issue_to_exe_valid,
    output wire issue_to_exe_valid1,
    input  wire exe_allow_in,

    input  wire [`EXE_TO_ID_BYPASS_BUS_WIDTH-1:0] exe_to_issue_bus,
    input  wire [`EXE_TO_ID_BYPASS_BUS_WIDTH-1:0] exe_to_issue_bus1,
    input  wire [`MEM_TO_ID_BYPASS_BUS_WIDTH-1:0] mem_to_issue_bus,
    input  wire [`MEM_TO_ID_BYPASS_BUS_WIDTH-1:0] mem_to_issue_bus1,
    input  wire [`WB_TO_ID_BUS_WIDTH-1:0] wb_to_issue_bus,
    input  wire [`WB_TO_ID_BUS_WIDTH-1:0] wb_to_issue_bus1,
    input  wire exe_flush,
    input  wire wb_ex,
    input  wire mret_flush,

    output wire [1023:0] debug_gpr_flat,
    output wire perf_load_use_stall
);
    //============================================================
    // pipeline control and stage registers
    //============================================================

    // wb_flush：WB级发现异常或mret时，后面所有年轻指令都作废。
    // 可以把它理解成最终裁判吹哨，ISSUE里排队的票不能再发车。
    wire wb_flush = wb_ex || mret_flush;

    // issue_valid：slot0这张票是不是真指令；为0时只是气泡。
    // issue_valid1：slot1这张票是不是真指令；它一定比slot0年轻，且必须跟着slot0一起存在。
    // issue_reg/issue_reg1：ID送来的两个“大行李箱”，里面装完整译码控制包。
    // issue_rf_reg：ID另送的“小纸条”，只写四个源寄存器号和是否真的要读。
    reg issue_valid;
    reg issue_valid1;
    reg [`ID_TO_ISSUE_BUS_WIDTH-1:0] issue_reg;
    reg [`ID_TO_ISSUE_BUS_WIDTH-1:0] issue_reg1;
    reg [`ID_TO_ISSUE_RF_BUS_WIDTH-1:0] issue_rf_reg;

    // issue_ready_go：ISSUE自己有没有准备好。为0通常表示前面的load/MUL/late结果还没回来。
    // id_allow_in：告诉ID“我这里能收下一组了”。
    // issue_to_exe_valid/valid1：告诉EXE“这两个槽现在有效，可以执行”。
    wire issue_ready_go;
    assign id_allow_in = !issue_valid || (issue_ready_go && exe_allow_in);
    assign issue_to_exe_valid = issue_valid && issue_ready_go &&
                                !exe_flush && !wb_flush;
    assign issue_to_exe_valid1 = issue_valid1 && issue_ready_go &&
                                 !exe_flush && !wb_flush;

    always @(posedge clk) begin
        if (reset || exe_flush || wb_flush) begin
            issue_valid  <= 1'b0;
            issue_valid1 <= 1'b0;
        end else if (id_allow_in) begin
            issue_valid  <= id_to_issue_valid;
            issue_valid1 <= id_to_issue_valid && id_to_issue_valid1;
        end
    end

    always @(posedge clk) begin
        if (id_to_issue_valid && id_allow_in) begin
            issue_reg    <= id_to_issue_bus;
            issue_reg1   <= id_to_issue_bus1;
            issue_rf_reg <= id_to_issue_rf_bus;
        end
    end

    //============================================================
    // source-register bus and bypass-bus unpack
    //============================================================

    // use_rs10/use_rs20：slot0的rs1/rs2是否真的需要读寄存器。
    // use_rs11/use_rs21：slot1的rs1/rs2是否真的需要读寄存器。
    // rs10/rs20/rs11/rs21：四个源寄存器号；10=slot0源1，20=slot0源2，11/21同理。
    wire use_rs10, use_rs20, use_rs11, use_rs21;
    wire [4:0] rs10, rs20, rs11, rs21;
    assign {use_rs10, use_rs20, rs10, rs20,
            use_rs11, use_rs21, rs11, rs21} = issue_rf_reg;

    // wb_*：WB级刚要写回的答案。寄存器堆可能还没来得及被读出最新值，
    // 所以ISSUE直接从WB旁路拿答案。valid=这槽有效，we=会写rd，rd=目的寄存器号，data=要写回的值。
    wire wb_valid, wb_we, wb_valid1, wb_we1;
    wire [4:0] wb_rd, wb_rd1;
    wire [31:0] wb_data, wb_data1;
    assign {wb_valid, wb_we, wb_rd, wb_data} = wb_to_issue_bus;
    assign {wb_valid1, wb_we1, wb_rd1, wb_data1} = wb_to_issue_bus1;

    // exe_*：EXE级离当前消费者最近，所以旁路优先级最高。
    // exe_load=生产者是load；exe_is_lw=它是完整32位LW，可在下一拍用原始DRAM返回值补救；
    // exe_late=结果还晚到，不能立刻前递；exe_br=分支纠错标记，这里只收拢不用。
    wire exe_valid, exe_we, exe_load, exe_is_lw, exe_late, exe_br;
    wire exe_valid1, exe_we1, exe_load1, exe_is_lw1, exe_late1, exe_br1;
    wire [4:0] exe_rd, exe_rd1;
    wire [31:0] exe_data, exe_data1;
    assign {exe_valid, exe_we, exe_rd, exe_data,
            exe_load, exe_is_lw, exe_late, exe_br} = exe_to_issue_bus;
    assign {exe_valid1, exe_we1, exe_rd1, exe_data1,
            exe_load1, exe_is_lw1, exe_late1, exe_br1} = exe_to_issue_bus1;

    // mem_*：MEM级已经走过EXE一拍，普通ALU/load/MUL最终结果多数在这里成熟，可给ISSUE前递。
    // 命名规则和WB一样：valid看有没有，we看写不写rd，rd是寄存器号，data是答案。
    wire mem_valid, mem_we, mem_valid1, mem_we1;
    wire [4:0] mem_rd, mem_rd1;
    wire [31:0] mem_data, mem_data1;
    assign {mem_valid, mem_we, mem_rd, mem_data} = mem_to_issue_bus;
    assign {mem_valid1, mem_we1, mem_rd1, mem_data1} = mem_to_issue_bus1;

    //============================================================
    // four-read/two-write register file
    //============================================================

    // rf_data0/rf_data1：寄存器堆读出的slot0两个源操作数。
    // rf_data2/rf_data3：寄存器堆读出的slot1两个源操作数。
    // 它们是“默认答案”；如果前面流水级有更新答案命中，就会被fwd函数替换。
    wire [31:0] rf_data0, rf_data1, rf_data2, rf_data3;
    regfile u_regfile (
        .clk(clk), .reset(reset),
        .raddr0(rs10), .rdata0(rf_data0),
        .raddr1(rs20), .rdata1(rf_data1),
        .raddr2(rs11), .rdata2(rf_data2),
        .raddr3(rs21), .rdata3(rf_data3),
        .we0(wb_valid && wb_we), .waddr0(wb_rd), .wdata0(wb_data),
        // [阶段4修改] slot1 WB接通第二逻辑写银行；同rd仍由年轻写口1优先。
        .we1(wb_valid1 && wb_we1), .waddr1(wb_rd1), .wdata1(wb_data1),
        .debug_gpr_flat(debug_gpr_flat)
    );

    //============================================================
    // forwarding priority and operand selection
    //============================================================

    // [阶段4修改] 每个源只查看三个相邻流水级的两个局部槽。
    // 级间优先级为EXE > MEM > WB > RF；同一级内slot1较年轻，放在slot0前。
    // 配对器禁止同组WAW，正常情况下两个同级槽不会同时命中同一个src。
    function [31:0] fwd;
        // fwd就是旁路选择器。src是要读的寄存器号，rf是寄存器堆给出的默认旧值。
        // e0/e1、m0/m1、w0/w1分别代表EXE、MEM、WB三个阶段里的slot0/slot1。
        // 每组参数里：v=有效，w=会写rd，l=结果晚到不能用，d=rd号，x=可前递的数据。
        input [4:0] src;
        input [31:0] rf;
        input e0v, e0w, e0l;
        input [4:0] e0d;
        input [31:0] e0x;
        input e1v, e1w, e1l;
        input [4:0] e1d;
        input [31:0] e1x;
        input m0v, m0w;
        input [4:0] m0d;
        input [31:0] m0x;
        input m1v, m1w;
        input [4:0] m1d;
        input [31:0] m1x;
        input w0v, w0w;
        input [4:0] w0d;
        input [31:0] w0x;
        input w1v, w1w;
        input [4:0] w1d;
        input [31:0] w1x;
        // e0h/e1h/m0h/m1h/w0h/w1h中的h表示hit：这一站刚好能给src这个寄存器答案。
        // ex_value/mem_value/wb_value先在同一级内挑出该用的值，再按EXE>MEM>WB>RF选最终值。
        reg e0h, e1h, m0h, m1h, w0h, w1h;
        reg [31:0] ex_value, mem_value, wb_value;
        begin
            // [阶段4时序优化] 两个槽的比较完全并行，同一级先局部归约，
            // 最后只保留EXE/MEM/WB三级优先选择，避免六层串行旁路MUX。
            e0h = e0v && e0w && !e0l && e0d == src;
            e1h = e1v && e1w && !e1l && e1d == src;
            m0h = m0v && m0w && m0d == src;
            m1h = m1v && m1w && m1d == src;
            w0h = w0v && w0w && w0d == src;
            w1h = w1v && w1w && w1d == src;
            ex_value  = e1h ? e1x : e0x;
            mem_value = m1h ? m1x : m0x;
            wb_value  = w1h ? w1x : w0x;

            if (src == 5'b0)
                fwd = 32'b0;
            else if (e0h || e1h)
                fwd = ex_value;
            else if (m0h || m1h)
                fwd = mem_value;
            else if (w0h || w1h)
                fwd = wb_value;
            else
                fwd = rf;
        end
    endfunction

    // rs1_value0/rs2_value0：slot0最终送给EXE的两个操作数。
    // rs1_value1/rs2_value1：slot1最终送给EXE的两个操作数。
    // 到这里已经完成寄存器堆读取和全部旁路选择，EXE只管算。
    wire [31:0] rs1_value0 = fwd(
        rs10, rf_data0,
        exe_valid, exe_we, exe_late, exe_rd, exe_data,
        exe_valid1, exe_we1, exe_late1, exe_rd1, exe_data1,
        mem_valid, mem_we, mem_rd, mem_data,
        mem_valid1, mem_we1, mem_rd1, mem_data1,
        wb_valid, wb_we, wb_rd, wb_data,
        wb_valid1, wb_we1, wb_rd1, wb_data1);
    wire [31:0] rs2_value0 = fwd(
        rs20, rf_data1,
        exe_valid, exe_we, exe_late, exe_rd, exe_data,
        exe_valid1, exe_we1, exe_late1, exe_rd1, exe_data1,
        mem_valid, mem_we, mem_rd, mem_data,
        mem_valid1, mem_we1, mem_rd1, mem_data1,
        wb_valid, wb_we, wb_rd, wb_data,
        wb_valid1, wb_we1, wb_rd1, wb_data1);
    wire [31:0] rs1_value1 = fwd(
        rs11, rf_data2,
        exe_valid, exe_we, exe_late, exe_rd, exe_data,
        exe_valid1, exe_we1, exe_late1, exe_rd1, exe_data1,
        mem_valid, mem_we, mem_rd, mem_data,
        mem_valid1, mem_we1, mem_rd1, mem_data1,
        wb_valid, wb_we, wb_rd, wb_data,
        wb_valid1, wb_we1, wb_rd1, wb_data1);
    wire [31:0] rs2_value1 = fwd(
        rs21, rf_data3,
        exe_valid, exe_we, exe_late, exe_rd, exe_data,
        exe_valid1, exe_we1, exe_late1, exe_rd1, exe_data1,
        mem_valid, mem_we, mem_rd, mem_data,
        mem_valid1, mem_we1, mem_rd1, mem_data1,
        wb_valid, wb_we, wb_rd, wb_data,
        wb_valid1, wb_we1, wb_rd1, wb_data1);

    //============================================================
    // load-use and late-result hazards
    //============================================================

    // [阶段4修复] 显式写出双槽load命中，避免组合函数读取外部信号时
    // 开源仿真器漏掉敏感项；四个源各自只做两次5位相等比较。
    // load_hit10/20/11/21：四个源寄存器有没有撞上EXE里正在做load的rd。
    // 撞上就像“答案还在内存路上”，通常要停一下，除非后面的LW短路能救。
    wire load_hit10 = use_rs10 && rs10 != 5'b0 &&
                      ((exe_valid && exe_we && exe_load && exe_rd == rs10) ||
                       (exe_valid1 && exe_we1 && exe_load1 && exe_rd1 == rs10));
    wire load_hit20 = use_rs20 && rs20 != 5'b0 &&
                      ((exe_valid && exe_we && exe_load && exe_rd == rs20) ||
                       (exe_valid1 && exe_we1 && exe_load1 && exe_rd1 == rs20));
    wire load_hit11 = issue_valid1 && use_rs11 && rs11 != 5'b0 &&
                      ((exe_valid && exe_we && exe_load && exe_rd == rs11) ||
                       (exe_valid1 && exe_we1 && exe_load1 && exe_rd1 == rs11));
    wire load_hit21 = issue_valid1 && use_rs21 && rs21 != 5'b0 &&
                      ((exe_valid && exe_we && exe_load && exe_rd == rs21) ||
                       (exe_valid1 && exe_we1 && exe_load1 && exe_rd1 == rs21));
    // load_stall：传统load-use总命中标记；当前真正停不停还要看ld短标签能不能修补。
    wire load_stall = load_hit10 || load_hit20 || load_hit11 || load_hit21;

    // [阶段5C2新增] 普通整数ALU消费者只包括RV32I OP-IMM和非M扩展OP。
    // 消费者分类和rd比较全部在ISSUE完成，EXE只读取寄存后的ld1/ld2，
    // 因此这些译码信号不会串入DRAM返回值到ALU的数据关键路径。
    // inst0/inst1：从控制包里重新拿出原始指令字，只为判断消费者是不是普通整数ALU。
    // alu0/alu1为1时，说明这条消费者可以在EXE用LW返回值直接替换操作数。
    wire [31:0] inst0 = issue_reg[338:307];
    wire [31:0] inst1 = issue_reg1[338:307];
    wire alu0 = (inst0[6:0] == 7'b0010011) ||
                ((inst0[6:0] == 7'b0110011) &&
                 (inst0[31:25] != 7'b0000001));
    wire alu1 = (inst1[6:0] == 7'b0010011) ||
                ((inst1[6:0] == 7'b0110011) &&
                 (inst1[31:25] != 7'b0000001));

    // 四个LW命中比较彼此并行。配对器禁止同组双LSU和WAW，因此同一个
    // 消费源不会同时匹配两个不同LW生产者。
    // lw_hit10/20/11/21：比load_hit更窄，只认完整32位LW。
    // 窄load还要在MEM做字节抽取和符号扩展，不能用原始32位内存返回值直接补。
    wire lw_hit10 = use_rs10 && rs10 != 5'b0 &&
                    ((exe_valid && exe_we && exe_is_lw && exe_rd == rs10) ||
                     (exe_valid1 && exe_we1 && exe_is_lw1 && exe_rd1 == rs10));
    wire lw_hit20 = use_rs20 && rs20 != 5'b0 &&
                    ((exe_valid && exe_we && exe_is_lw && exe_rd == rs20) ||
                     (exe_valid1 && exe_we1 && exe_is_lw1 && exe_rd1 == rs20));
    wire lw_hit11 = issue_valid1 && use_rs11 && rs11 != 5'b0 &&
                    ((exe_valid && exe_we && exe_is_lw && exe_rd == rs11) ||
                     (exe_valid1 && exe_we1 && exe_is_lw1 && exe_rd1 == rs11));
    wire lw_hit21 = issue_valid1 && use_rs21 && rs21 != 5'b0 &&
                    ((exe_valid && exe_we && exe_is_lw && exe_rd == rs21) ||
                     (exe_valid1 && exe_we1 && exe_is_lw1 && exe_rd1 == rs21));

    // ld1/ld2是随消费者寄存到EXE的两位短标签。只有“LW生产者 + 普通
    // 整数ALU消费者”才置位；MUL、branch、store和窄load不会进入此路。
    // ld10/ld20/ld11/ld21：写进ISSUE→EXE总线最前面的四张“小纸条”。
    // 例如ld10=1，就是告诉EXE“slot0的rs1下拍用data_sram_rdata替换”。
    wire ld10 = alu0 && lw_hit10;
    wire ld20 = alu0 && lw_hit20;
    wire ld11 = issue_valid1 && alu1 && lw_hit11;
    wire ld21 = issue_valid1 && alu1 && lw_hit21;

    // [阶段5A新增] late比load含义更宽：EXE中的load和MUL都还没有最终
    // 写回值。四个源比较完全并行，只在最后做一次归约，不增加串行MUX层数。
    // late_hit10/20/11/21：四个源有没有撞上“答案晚到”的生产者。
    // late范围比load大：load、MUL、后置ALU都可能让答案这拍还不能用。
    wire late_hit10 = use_rs10 && rs10 != 5'b0 &&
                      ((exe_valid && exe_we && exe_late && exe_rd == rs10) ||
                       (exe_valid1 && exe_we1 && exe_late1 && exe_rd1 == rs10));
    wire late_hit20 = use_rs20 && rs20 != 5'b0 &&
                      ((exe_valid && exe_we && exe_late && exe_rd == rs20) ||
                       (exe_valid1 && exe_we1 && exe_late1 && exe_rd1 == rs20));
    wire late_hit11 = issue_valid1 && use_rs11 && rs11 != 5'b0 &&
                      ((exe_valid && exe_we && exe_late && exe_rd == rs11) ||
                       (exe_valid1 && exe_we1 && exe_late1 && exe_rd1 == rs11));
    wire late_hit21 = issue_valid1 && use_rs21 && rs21 != 5'b0 &&
                      ((exe_valid && exe_we && exe_late && exe_rd == rs21) ||
                       (exe_valid1 && exe_we1 && exe_late1 && exe_rd1 == rs21));
    // [阶段5C2修改] 只从late停顿中扣除允许修复的LW命中。若同一条消费
    // 指令还有另一个MUL或非LW晚结果依赖，对应项仍为1，整组继续停顿。
    // late_stall：真正卡住ISSUE的总闸门。能被ld短标签修补的LW命中会扣掉，
    // 剩下的晚到依赖必须等到MEM/WB前递。
    // load_block：只给性能计数器看的“load-use确实挡路了”事件。
    wire late_stall = (late_hit10 && !ld10) ||
                      (late_hit20 && !ld20) ||
                      (late_hit11 && !ld11) ||
                      (late_hit21 && !ld21);
    wire load_block = (load_hit10 && !ld10) ||
                      (load_hit20 && !ld20) ||
                      (load_hit11 && !ld11) ||
                      (load_hit21 && !ld21);

    assign issue_ready_go = !late_stall;
    assign perf_load_use_stall = issue_valid && load_block &&
                                 !exe_flush && !wb_flush;

    // [阶段5C2修改] 两个新标签放在最高位，原374位控制包的所有字段保持
    // 原相对位置；两个槽仍只替换各自的rs1/rs2操作数64位。
    //============================================================
    // ISSUE to EXE buses
    //============================================================

    assign issue_to_exe_bus = {
        ld10, ld20,
        issue_reg[`ID_TO_ISSUE_BUS_WIDTH-1:281],
        rs1_value0, rs2_value0, issue_reg[216:0]
    };
    assign issue_to_exe_bus1 = {
        ld11, ld21,
        issue_reg1[`ID_TO_ISSUE_BUS_WIDTH-1:281],
        rs1_value1, rs2_value1, issue_reg1[216:0]
    };

    // unused_branch：把当前版本暂时不用的exe_br/exe_br1/load_stall收拢，避免lint告警。
    // 它没有功能含义，不参与流水线控制。
    wire unused_branch = exe_br || exe_br1 || load_stall;
endmodule

`timescale 1ns / 1ps

module tb_student_top_enhanced;

    // ------------------------------------------------------------
    // 参数区（按需改）
    // ------------------------------------------------------------
    localparam int P_SW_CNT  = 64;
    localparam int P_LED_CNT = 32;
    localparam int P_SEG_CNT = 40;
    localparam int P_KEY_CNT = 8;

    localparam time CPU_CLK_HALF = 5ns;    // 100MHz
    localparam time CNT_CLK_HALF = 10ns;   // 50MHz

    localparam int  RESET_CYCLES   = 40;
    localparam int  TIMEOUT_CYCLES = 2_000_000;

    // 结束条件
    localparam bit         END_BY_PC_EN      = 1'b0;
    localparam logic [31:0] END_PC           = 32'h8000_0100;
    localparam bit         END_BY_EBREAK_EN  = 1'b1;   // instr == 32'h00100073

    // trace 输出文件
    localparam string TRACE_FILE = "riscv_trace.txt";

    // 可选：仿龙芯 num_monitor（默认关闭）
    localparam bit         NUM_MONITOR_EN   = 1'b0;
    localparam logic [31:0] NUM_MONITOR_ADDR = 32'h8020_0040; // 默认 LED_ADDR

    // 可选：模拟 UART 打印（默认关闭，当前 perip_bridge 未定义 UART）
    localparam bit         UART_MONITOR_EN  = 1'b0;
    localparam logic [31:0] UART_ADDR        = 32'h8020_0060;
    localparam logic [7:0]  UART_END_DATA    = 8'hff;

    // perip_bridge 地址映射（来自你的 perip_bridge.sv）
    localparam logic [31:0] DRAM_ADDR_START = 32'h8010_0000;
    localparam logic [31:0] DRAM_ADDR_END   = 32'h8013_FFFF;
    localparam logic [31:0] SW0_ADDR        = 32'h8020_0000;
    localparam logic [31:0] SW1_ADDR        = 32'h8020_0004;
    localparam logic [31:0] KEY_ADDR        = 32'h8020_0010;
    localparam logic [31:0] SEG_ADDR        = 32'h8020_0020;
    localparam logic [31:0] LED_ADDR        = 32'h8020_0040;
    localparam logic [31:0] CNT_ADDR        = 32'h8020_0050;

    // ------------------------------------------------------------
    // DUT IO
    // ------------------------------------------------------------
    logic                    w_cpu_clk;
    logic                    w_clk_50Mhz;
    logic                    w_clk_rst;
    logic [P_KEY_CNT-1:0]    virtual_key;
    logic [P_SW_CNT-1:0]     virtual_sw;
    wire  [P_LED_CNT-1:0]    virtual_led;
    wire  [P_SEG_CNT-1:0]    virtual_seg;

    student_top #(
        .P_SW_CNT   (P_SW_CNT),
        .P_LED_CNT  (P_LED_CNT),
        .P_SEG_CNT  (P_SEG_CNT),
        .P_KEY_CNT  (P_KEY_CNT)
    ) u_dut (
        .w_cpu_clk   (w_cpu_clk),
        .w_clk_50Mhz (w_clk_50Mhz),
        .w_clk_rst   (w_clk_rst),
        .virtual_key (virtual_key),
        .virtual_sw  (virtual_sw),
        .virtual_led (virtual_led),
        .virtual_seg (virtual_seg)
    );

    // ------------------------------------------------------------
    // 时钟
    // ------------------------------------------------------------
    initial w_cpu_clk = 1'b0;
    always #(CPU_CLK_HALF) w_cpu_clk = ~w_cpu_clk;

    initial w_clk_50Mhz = 1'b0;
    always #(CNT_CLK_HALF) w_clk_50Mhz = ~w_clk_50Mhz;

    // ------------------------------------------------------------
    // 复位+基础激励
    // ------------------------------------------------------------
    initial begin
        w_clk_rst   = 1'b1; // 高有效复位
        virtual_sw  = 64'h0;
        virtual_key = 8'h0;

        repeat (RESET_CYCLES) @(posedge w_cpu_clk);
        w_clk_rst = 1'b0;

        // 例子激励：你可以按程序需求改
        repeat (100) @(posedge w_cpu_clk);
        virtual_sw <= 64'h0123_4567_89AB_CDEF;

        repeat (80) @(posedge w_cpu_clk);
        virtual_key[0] <= 1'b1;
        repeat (2) @(posedge w_cpu_clk);
        virtual_key[0] <= 1'b0;

        repeat (200) @(posedge w_cpu_clk);
        virtual_sw <= 64'hFEDC_BA98_7654_3210;
    end

    // ------------------------------------------------------------
    // 取“龙芯风格 debug_wb_*”等价信号（层次引用）
    // ------------------------------------------------------------
    wire        soc_clk           = w_cpu_clk;
    wire [31:0] debug_wb_pc       = u_dut.Core_cpu.pc;
    wire        debug_wb_rf_we    = u_dut.Core_cpu.RegWrite;
    wire [4:0]  debug_wb_rf_wnum  = u_dut.Core_cpu.instr[11:7];
    wire [31:0] debug_wb_rf_wdata = u_dut.Core_cpu.wdata;
    wire [3:0]  debug_wb_rf_wen   = {4{debug_wb_rf_we}};
    wire [31:0] debug_wb_rf_wdata_v = debug_wb_rf_wdata; // RV32 寄存器整字写

    // ------------------------------------------------------------
    // 仿真控制
    // ------------------------------------------------------------
    integer trace_fd;
    integer cycle_cnt;
    integer err_count;
    bit     sim_done;

    task automatic sim_finish(input bit pass, input string msg);
        begin
            if (!sim_done) begin
                sim_done = 1'b1;
                $display("==============================================================");
                if (pass) $display("PASS: %s", msg);
                else      $display("FAIL: %s", msg);
                $display("cycle=%0d, err_count=%0d, pc=0x%08h", cycle_cnt, err_count, debug_wb_pc);
                $display("==============================================================");
                if (trace_fd) $fclose(trace_fd);
                #20;
                $finish;
            end
        end
    endtask

    initial begin
        sim_done  = 1'b0;
        cycle_cnt = 0;
        err_count = 0;

        trace_fd = $fopen(TRACE_FILE, "w");
        if (trace_fd == 0) begin
            $display("ERROR: cannot open %s", TRACE_FILE);
            $finish;
        end
    end

    // ------------------------------------------------------------
    // 1) 生成 trace（对齐龙芯风格：open_trace pc wnum wdata）
    // ------------------------------------------------------------
    // 这里 open_trace 固定写 1（工程里没有 confreg.open_trace）
    always @(posedge soc_clk) begin
        if (!w_clk_rst && !sim_done) begin
            if (|debug_wb_rf_wen && (debug_wb_rf_wnum != 5'd0)) begin
                $fdisplay(trace_fd, "%h %h %h %h",
                          32'h0000_0001,
                          debug_wb_pc,
                          {27'd0, debug_wb_rf_wnum},
                          debug_wb_rf_wdata_v);
            end
        end
    end

    // ------------------------------------------------------------
    // 2) 外设读一致性检查（SW/KEY）
    // ------------------------------------------------------------
    always @(posedge soc_clk) begin
        if (!w_clk_rst && !sim_done) begin
            if (!u_dut.perip_wen) begin
                case (u_dut.perip_addr)
                    SW0_ADDR: if (u_dut.perip_rdata !== virtual_sw[31:0]) begin
                        err_count <= err_count + 1;
                        $display("[%t] ERR SW0 read mismatch: got=%h exp=%h",
                                 $time, u_dut.perip_rdata, virtual_sw[31:0]);
                    end
                    SW1_ADDR: if (u_dut.perip_rdata !== virtual_sw[63:32]) begin
                        err_count <= err_count + 1;
                        $display("[%t] ERR SW1 read mismatch: got=%h exp=%h",
                                 $time, u_dut.perip_rdata, virtual_sw[63:32]);
                    end
                    KEY_ADDR: if (u_dut.perip_rdata !== {24'd0, virtual_key}) begin
                        err_count <= err_count + 1;
                        $display("[%t] ERR KEY read mismatch: got=%h exp=%h",
                                 $time, u_dut.perip_rdata, {24'd0, virtual_key});
                    end
                    default: begin end
                endcase
            end
        end
    end

    // ------------------------------------------------------------
    // 3) LED 写后一致性检查（下一拍检查）
    // ------------------------------------------------------------
    logic       led_chk_pending;
    logic [31:0] led_expected;

    always @(posedge soc_clk) begin
        if (w_clk_rst) begin
            led_chk_pending <= 1'b0;
            led_expected    <= 32'h0;
        end else if (!sim_done) begin
            if (led_chk_pending) begin
                if (virtual_led !== led_expected) begin
                    err_count <= err_count + 1;
                    $display("[%t] ERR LED mismatch: got=%h exp=%h",
                             $time, virtual_led, led_expected);
                end
                led_chk_pending <= 1'b0;
            end

            if (u_dut.perip_wen && (u_dut.perip_addr == LED_ADDR)) begin
                led_expected    <= u_dut.perip_wdata;
                led_chk_pending <= 1'b1;
            end
        end
    end

    // ------------------------------------------------------------
    // 4) 可选：功能点监控（仿龙芯 confreg_num_reg 递增检查）
    // ------------------------------------------------------------
    logic [31:0] num_reg_r;
    logic        num_reg_valid;

    always @(posedge soc_clk) begin
        if (w_clk_rst) begin
            num_reg_r     <= 32'd0;
            num_reg_valid <= 1'b0;
        end else if (!sim_done && NUM_MONITOR_EN) begin
            if (u_dut.perip_wen && (u_dut.perip_addr == NUM_MONITOR_ADDR)) begin
                if (num_reg_valid) begin
                    if (u_dut.perip_wdata[7:0] != num_reg_r[7:0] + 1'b1) begin
                        err_count <= err_count + 1;
                        $display("--------------------------------------------------------------");
                        $display("[%t] Error(%0d)! num low8 not +1, got=%02h prev=%02h",
                                 $time, err_count, u_dut.perip_wdata[7:0], num_reg_r[7:0]);
                        $display("--------------------------------------------------------------");
                    end
                    else if (u_dut.perip_wdata[31:24] != num_reg_r[31:24] + 1'b1) begin
                        err_count <= err_count + 1;
                        $display("--------------------------------------------------------------");
                        $display("[%t] Error(%0d)! testpoint idx not +1, got=%02h prev=%02h",
                                 $time, err_count, u_dut.perip_wdata[31:24], num_reg_r[31:24]);
                        $display("--------------------------------------------------------------");
                    end
                    else begin
                        $display("----[%t] Functional Test Point %0d PASS",
                                 $time, u_dut.perip_wdata[31:24]);
                    end
                end
                num_reg_r     <= u_dut.perip_wdata;
                num_reg_valid <= 1'b1;
            end
        end
    end

    // ------------------------------------------------------------
    // 5) 可选：UART 打印（默认关闭）
    // ------------------------------------------------------------
    always @(posedge soc_clk) begin
        if (!w_clk_rst && !sim_done && UART_MONITOR_EN) begin
            if (u_dut.perip_wen && (u_dut.perip_addr == UART_ADDR)) begin
                if (u_dut.perip_wdata[7:0] == UART_END_DATA) begin
                    sim_finish((err_count == 0), "UART end flag received");
                end else begin
                    $write("%c", u_dut.perip_wdata[7:0]);
                end
            end
        end
    end

    // ------------------------------------------------------------
    // 6) 运行日志 + 结束条件 + 超时
    // ------------------------------------------------------------
    initial begin
        $timeformat(-9, 0, " ns", 10);
        while (w_clk_rst) @(posedge soc_clk);
        $display("==============================================================");
        $display("Test begin!");
        $display("==============================================================");
    end

    always @(posedge soc_clk) begin
        if (!w_clk_rst && !sim_done) begin
            cycle_cnt <= cycle_cnt + 1;

            if ((cycle_cnt % 10000) == 0) begin
                $display("[%t] running... pc=0x%08h instr=0x%08h",
                         $time, u_dut.pc, u_dut.instruction);
            end

            // END_PC
            if (END_BY_PC_EN && (debug_wb_pc == END_PC)) begin
                sim_finish((err_count == 0), $sformatf("reach END_PC=0x%08h", END_PC));
            end

            // EBREAK 结束
            if (END_BY_EBREAK_EN && (u_dut.instruction == 32'h0010_0073)) begin
                sim_finish((err_count == 0), "EBREAK detected");
            end

            // 超时保护
            if (cycle_cnt >= TIMEOUT_CYCLES) begin
                sim_finish(1'b0, "timeout");
            end
        end
    end

endmodule
`timescale 1ns / 1ps

`define TRACE_REF_FILE     "riscv_trace.txt"

// 结束条件（按需改）
`define USE_END_PC         1'b0
`define END_PC             32'h8000_0100
`define USE_EBREAK_END     1'b1

// 超时保护
`define TIMEOUT_CYCLES     2000000

// 仿 confreg_num_monitor（按需开）
`define NUM_MONITOR_EN     1'b0
`define NUM_MONITOR_ADDR   32'h8020_0040   // 默认用 LED_ADDR 作为功能点写口

// 模拟 UART（当前工程默认无该寄存器，保留开关）
`define UART_MONITOR_EN    1'b0
`define UART_ADDR          32'h8020_0060
`define UART_END_DATA      8'hff

module tb_top_riscv_custom();

reg         w_cpu_clk;
reg         w_clk_50Mhz;
reg         w_clk_rst;
reg  [7:0]  virtual_key;
reg  [63:0] virtual_sw;
wire [31:0] virtual_led;
wire [39:0] virtual_seg;

// --------------------------------------------------
// DUT
// --------------------------------------------------
student_top dut (
    .w_cpu_clk    (w_cpu_clk),
    .w_clk_50Mhz  (w_clk_50Mhz),
    .w_clk_rst    (w_clk_rst),
    .virtual_key  (virtual_key),
    .virtual_sw   (virtual_sw),
    .virtual_led  (virtual_led),
    .virtual_seg  (virtual_seg)
);

// --------------------------------------------------
// clock & reset
// --------------------------------------------------
initial begin
    w_cpu_clk   = 1'b0;
    w_clk_50Mhz = 1'b0;
    w_clk_rst   = 1'b1;    // 高有效复位
    virtual_sw  = 64'h0;
    virtual_key = 8'h0;
    #2000;
    w_clk_rst   = 1'b0;
end

always #5  w_cpu_clk   = ~w_cpu_clk;    // 100MHz
always #10 w_clk_50Mhz = ~w_clk_50Mhz;  // 50MHz

// 可选输入激励
initial begin
    wait(w_clk_rst == 1'b0);
    repeat (100) @(posedge w_cpu_clk);
    virtual_sw <= 64'h0123_4567_89AB_CDEF;

    repeat (100) @(posedge w_cpu_clk);
    virtual_key[0] <= 1'b1;
    repeat (2) @(posedge w_cpu_clk);
    virtual_key[0] <= 1'b0;
end

// --------------------------------------------------
// "soc lite signals" equivalent
// --------------------------------------------------
wire        soc_clk;
wire [31:0] debug_wb_pc;
wire [3 :0] debug_wb_rf_wen;
wire [4 :0] debug_wb_rf_wnum;
wire [31:0] debug_wb_rf_wdata;

assign soc_clk           = w_cpu_clk;

// 基于 myCPU 内部信号映射（你的工程可见）
assign debug_wb_pc       = dut.Core_cpu.pc;
assign debug_wb_rf_wen   = {4{dut.Core_cpu.RegWrite}};
assign debug_wb_rf_wnum  = dut.Core_cpu.instr[11:7];
assign debug_wb_rf_wdata = dut.Core_cpu.wdata;

// 字节有效屏蔽（保留龙芯 tb_top 同款写法）
wire [31:0] debug_wb_rf_wdata_v;
assign debug_wb_rf_wdata_v[31:24] = debug_wb_rf_wdata[31:24] & {8{debug_wb_rf_wen[3]}};
assign debug_wb_rf_wdata_v[23:16] = debug_wb_rf_wdata[23:16] & {8{debug_wb_rf_wen[2]}};
assign debug_wb_rf_wdata_v[15: 8] = debug_wb_rf_wdata[15: 8] & {8{debug_wb_rf_wen[1]}};
assign debug_wb_rf_wdata_v[7 : 0] = debug_wb_rf_wdata[7 : 0] & {8{debug_wb_rf_wen[0]}};

// --------------------------------------------------
// trace file
// --------------------------------------------------
integer trace_ref;
initial begin
    trace_ref = $fopen(`TRACE_REF_FILE, "w");
    if (trace_ref == 0) begin
        $display("ERROR: cannot open trace file: %s", `TRACE_REF_FILE);
        $finish;
    end
end

reg debug_end;

// generate trace (龙芯风格 4列)
always @(posedge soc_clk) begin
    if (!w_clk_rst && (|debug_wb_rf_wen) && (debug_wb_rf_wnum != 5'd0)) begin
        $fdisplay(trace_ref, "%h %h %h %h",
            32'h0000_0001,            // open_trace 等价位
            debug_wb_pc,
            {27'd0, debug_wb_rf_wnum},
            debug_wb_rf_wdata_v
        );
    end
end

// --------------------------------------------------
// monitor numeric display (confreg_num_monitor equivalent)
// --------------------------------------------------
reg [7:0]  err_count;
reg [31:0] confreg_num_reg;
reg [31:0] confreg_num_reg_r;

always @(posedge soc_clk) begin
    confreg_num_reg_r <= confreg_num_reg;

    if (w_clk_rst) begin
        err_count       <= 8'd0;
        confreg_num_reg <= 32'd0;
    end
    else begin
        if (`NUM_MONITOR_EN && dut.perip_wen && (dut.perip_addr == `NUM_MONITOR_ADDR)) begin
            confreg_num_reg <= dut.perip_wdata;
        end

        if (`NUM_MONITOR_EN && (confreg_num_reg_r != confreg_num_reg)) begin
            if (confreg_num_reg[7:0] != confreg_num_reg_r[7:0] + 1'b1) begin
                $display("--------------------------------------------------------------");
                $display("[%t] Error(%0d) low8 not +1 at testpoint %0d",
                         $time, err_count, confreg_num_reg[31:24]);
                $display("--------------------------------------------------------------");
                err_count <= err_count + 1'b1;
            end
            else if (confreg_num_reg[31:24] != confreg_num_reg_r[31:24] + 1'b1) begin
                $display("--------------------------------------------------------------");
                $display("[%t] Error(%0d) testpoint index not +1", $time, err_count);
                $display("--------------------------------------------------------------");
                err_count <= err_count + 1'b1;
            end
            else begin
                $display("----[%t] Functional Test Point %0d PASS",
                         $time, confreg_num_reg[31:24]);
            end
        end
    end
end

// --------------------------------------------------
// monitor test progress
// --------------------------------------------------
initial begin
    $timeformat(-9, 0, " ns", 10);
    while (w_clk_rst) #5;
    $display("==============================================================");
    $display("Test begin!");
    $display("==============================================================");

    #10000;
    while (!debug_end) begin
        #10000;
        $display("[%t] Test running, debug_wb_pc = 0x%08h", $time, debug_wb_pc);
    end
end

// --------------------------------------------------
// UART print (optional)
// --------------------------------------------------
wire uart_display;
wire [7:0] uart_data;
assign uart_display = (`UART_MONITOR_EN) && dut.perip_wen && (dut.perip_addr == `UART_ADDR);
assign uart_data    = dut.perip_wdata[7:0];

always @(posedge soc_clk) begin
    if (!w_clk_rst && uart_display) begin
        if (uart_data == `UART_END_DATA) begin
            $finish;
        end
        else begin
            $write("%c", uart_data);
        end
    end
end

// --------------------------------------------------
// timeout
// --------------------------------------------------
integer cycle_cnt;
always @(posedge soc_clk) begin
    if (w_clk_rst) begin
        cycle_cnt <= 0;
    end
    else if (!debug_end) begin
        cycle_cnt <= cycle_cnt + 1;
        if (cycle_cnt >= `TIMEOUT_CYCLES) begin
            $display("==============================================================");
            $display("TIMEOUT! cycle=%0d, pc=0x%08h", cycle_cnt, debug_wb_pc);
            $display("==============================================================");
            $fclose(trace_ref);
            $finish;
        end
    end
end

// --------------------------------------------------
// test end
// --------------------------------------------------
wire test_end;
assign test_end = ((`USE_END_PC) && (debug_wb_pc == `END_PC)) ||
                  ((`USE_EBREAK_END) && (dut.instruction == 32'h0010_0073));

always @(posedge soc_clk) begin
    if (w_clk_rst) begin
        debug_end <= 1'b0;
    end
    else if (test_end && !debug_end) begin
        debug_end <= 1'b1;
        $display("==============================================================");
        $display("gettrace end!");
        #10;
        $fclose(trace_ref);

        if (err_count != 8'd0) begin
            $display("Fail in generating trace file! Total %0d errors!", err_count);
        end
        else begin
            $display("----Succeed in generating trace file!");
        end
        $finish;
    end
end

endmodule
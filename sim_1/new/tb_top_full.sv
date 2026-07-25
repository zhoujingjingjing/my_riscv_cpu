`timescale 1ns / 1ps

// 分赛区决赛完整数字孪生平台验收testbench。
// 本文件直接实例化比赛top，使用真实PLL、IROM、DRAM、CPU、外设桥、
// Twin Controller和UART。控制台只打印阶段性结果，不逐条打印退休指令。
module tb_top_full;
    localparam [31:0] END_PC = 32'h8000_0014;
    localparam [63:0] MAX_CPU_CYCLES = 64'd600_000_000;
    localparam [31:0] PASS_ADDR = 32'h8010_0000;
    localparam [31:0] FAIL_ADDR = 32'h8010_0004;
    localparam [31:0] TEST_COUNT_ADDR = 32'h8010_0030;
    localparam [31:0] SEG_ADDR = 32'h8020_0020;
    localparam [31:0] LED_ADDR = 32'h8020_0040;
    localparam [31:0] COUNTER_ADDR = 32'h8020_0050;
    localparam [31:0] EXPECTED_FINAL_LED = 32'h078b_7323;
    localparam integer UART_BIT_PERIOD_NS = 104_166;

    reg sys_clk_p;
    wire sys_clk_n = ~sys_clk_p;
    reg serial_rx;
    wire serial_tx;
    wire [31:0] virtual_led;
    wire [39:0] virtual_seg;

    reg [7:0] uart_status[0:17];
    integer byte_index;
    integer compare_errors;

    top uut (
        .i_sys_clk_p (sys_clk_p),
        .i_sys_clk_n (sys_clk_n),
        .i_uart_rx   (serial_rx),
        .o_uart_tx   (serial_tx),
        .virtual_led (virtual_led),
        .virtual_seg (virtual_seg)
    );

    // 板卡差分输入基准时钟固定为200MHz；CPU频率由PLL的clk_out2决定。
    initial begin
        sys_clk_p = 1'b0;
        forever #2.5 sys_clk_p = ~sys_clk_p;
    end

    wire cpu_clk = uut.cpu_clk;
    wire timer_ref_clk = uut.w_clk_50Mhz;
    wire pll_locked = uut.w_clk_rst;
    wire [7:0] retire_count =
        uut.student_top_inst.Core_cpu.retire_count;
    wire [31:0] retire_pc =
        uut.student_top_inst.Core_cpu.retire_pc;
    wire [31:0] retire_inst =
        uut.student_top_inst.Core_cpu.retire_inst;
    wire [31:0] retire_dnpc =
        uut.student_top_inst.Core_cpu.retire_dnpc;
    wire retire_mem_valid =
        uut.student_top_inst.Core_cpu.retire_mem_valid;
    wire retire_mem_store =
        uut.student_top_inst.Core_cpu.retire_mem_store;
    wire [31:0] retire_mem_addr =
        uut.student_top_inst.Core_cpu.retire_mem_addr;
    wire [31:0] retire_mem_wdata =
        uut.student_top_inst.Core_cpu.retire_mem_wdata;
    wire [63:0] perf_mcycle =
        uut.student_top_inst.Core_cpu.perf_mcycle;
    wire [63:0] perf_minstret =
        uut.student_top_inst.Core_cpu.perf_minstret;
    wire [63:0] perf_load_use_stall_count =
        uut.student_top_inst.Core_cpu.perf_load_use_stall_count;
    wire [63:0] perf_branch_count =
        uut.student_top_inst.Core_cpu.perf_branch_count;
    wire [63:0] perf_mispredict_count =
        uut.student_top_inst.Core_cpu.perf_mispredict_count;
    wire [31:0] raw_seg_value =
        uut.student_top_inst.bridge_inst.seg_wdata;
    wire counter_write =
        uut.student_top_inst.bridge_inst.counter_inst.cnt_wen;
    wire [31:0] counter_write_data =
        uut.student_top_inst.bridge_inst.counter_inst.perip_wdata;
    wire [31:0] counter_ms_value =
        uut.student_top_inst.bridge_inst.counter_inst.perip_rdata;

    reg cpu_done;
    reg [31:0] rv32i_pass_count;
    reg [31:0] final_test_count;
    reg [63:0] rv32m_retire_count;
    reg [63:0] next_progress_cycle;
    reg [63:0] cpi_x1e6;
    realtime cpu_execution_seconds;

    function automatic bcd5_is_valid(input [19:0] value);
        integer nibble;
        begin
            bcd5_is_valid = 1'b1;
            for (nibble = 0; nibble < 5; nibble = nibble + 1)
                if (value[nibble * 4 +: 4] > 4'd9)
                    bcd5_is_valid = 1'b0;
        end
    endfunction

    task automatic uart_send_byte(input [7:0] data);
        integer bit_index;
        begin
            serial_rx = 1'b0;
            #(UART_BIT_PERIOD_NS);
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
                serial_rx = data[bit_index];
                #(UART_BIT_PERIOD_NS);
            end
            serial_rx = 1'b1;
            #(UART_BIT_PERIOD_NS);
        end
    endtask

    task automatic uart_receive_byte(output [7:0] data);
        integer bit_index;
        begin
            wait (serial_tx === 1'b0);
            #(UART_BIT_PERIOD_NS / 2);
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
                #(UART_BIT_PERIOD_NS);
                data[bit_index] = serial_tx;
            end
            #(UART_BIT_PERIOD_NS);
        end
    endtask

    initial begin
        serial_rx = 1'b1;
        cpu_done = 1'b0;
        rv32i_pass_count = 32'b0;
        final_test_count = 32'b0;
        rv32m_retire_count = 64'b0;
        next_progress_cycle = 64'd100_000_000;
        cpi_x1e6 = 64'b0;
        cpu_execution_seconds = 0.0;
        compare_errors = 0;
        for (byte_index = 0; byte_index < 18; byte_index = byte_index + 1)
            uart_status[byte_index] = 8'b0;

        $timeformat(-9, 3, " ns", 16);
        $display("============================================================");
        $display("分赛区决赛完整平台仿真开始");
        $display("CPU频率         : 由当前PLL输出自动测量，不设固定值");
        $display("程序结束PC      : 0x%08h", END_PC);
        $display("期望最终LED     : 0x%08h", EXPECTED_FINAL_LED);
        $display("最大保险周期    : %0d", MAX_CPU_CYCLES);
        $display("============================================================");

        wait (pll_locked === 1'b1);
        $display("[%0t] PLL locked，CPU解除复位", $time);
        wait (cpu_done === 1'b1);
        @(posedge cpu_clk);
        #1;

        if (rv32i_pass_count !== 32'd37)
            $fatal(1, "RV32I通过数量错误：期望37，实际%0d", rv32i_pass_count);
        if (final_test_count !== 32'd8)
            $fatal(1, "决赛测试项数量错误：期望8，实际%0d", final_test_count);
        if (virtual_led !== EXPECTED_FINAL_LED)
            $fatal(1, "最终LED错误：期望0x%08h，实际0x%08h",
                   EXPECTED_FINAL_LED, virtual_led);
        if (raw_seg_value[31:20] !== 12'h378)
            $fatal(1, "数码管高三位错误：期望378，实际%03h",
                   raw_seg_value[31:20]);
        if (!bcd5_is_valid(raw_seg_value[19:0]))
            $fatal(1, "数码管时间不是合法BCD：0x%05h", raw_seg_value[19:0]);

        cpi_x1e6 = (perf_minstret == 0)
            ? 64'b0 : (perf_mcycle * 64'd1_000_000) / perf_minstret;
        cpu_execution_seconds = perf_mcycle * cpu_period_ns / 1.0e9;
        $display("");
        $display("================ CPU最终结果 ================");
        $display("RV32I通过数量     : %0d / 37", rv32i_pass_count);
        $display("决赛测试项数量    : %0d / 8", final_test_count);
        $display("RV32M退休数量     : %0d", rv32m_retire_count);
        $display("最终LED            : 0x%08h", virtual_led);
        $display("数码管原始BCD      : 0x%08h", raw_seg_value);
        $display("计时结果           : %05h ms", raw_seg_value[19:0]);
        $display("mcycle             : %0d", perf_mcycle);
        $display("minstret            : %0d", perf_minstret);
        $display("CPI                 : %0d.%06d",
                 cpi_x1e6 / 1_000_000, cpi_x1e6 % 1_000_000);
        $display("实测CPU频率        : %.3f MHz", 1000.0 / cpu_period_ns);
        $display("CPU执行时间        : %.9f s", cpu_execution_seconds);
        $display("load-use停顿       : %0d", perf_load_use_stall_count);
        $display("分支数量            : %0d", perf_branch_count);
        $display("预测错误数量        : %0d", perf_mispredict_count);

        $display("");
        $display("开始通过UART发送0x80读取数字孪生平台的18字节状态……");
        fork
            begin
                uart_send_byte(8'h80);
            end
            begin
                for (byte_index = 0; byte_index < 18; byte_index = byte_index + 1) begin
                    uart_receive_byte(uart_status[byte_index]);
                    $display("UART状态[%0d] = 0x%02h", byte_index,
                             uart_status[byte_index]);
                end
            end
        join

        for (byte_index = 0; byte_index < 18; byte_index = byte_index + 1) begin
            if (uart_status[byte_index] !==
                uut.twin_controller_inst.status_buffer[byte_index]) begin
                compare_errors = compare_errors + 1;
                $display("UART字节不一致[%0d]：收到0x%02h，缓冲区0x%02h",
                         byte_index, uart_status[byte_index],
                         uut.twin_controller_inst.status_buffer[byte_index]);
            end
        end
        if (compare_errors != 0)
            $fatal(1, "UART返回的18字节与Twin Controller缓冲区不一致：%0d处错误",
                   compare_errors);
        if ({uart_status[17], uart_status[16], uart_status[15], uart_status[14]}
            !== EXPECTED_FINAL_LED)
            $fatal(1, "UART返回的LED值错误：0x%08h",
                   {uart_status[17], uart_status[16],
                    uart_status[15], uart_status[14]});

        $display("UART返回的18字节与内部数字孪生状态完全一致");
        $display("============================================================");
        $display("FINAL_ROUND_TB_PASS");
        $display("============================================================");
        $finish;
    end

    // CPU频率只测量和打印，不限制为某个固定值；以后可按WNS调整clk_out2。
    realtime first_cpu_edge;
    realtime second_cpu_edge;
    realtime cpu_period_ns;
    initial begin
        wait (pll_locked === 1'b1);
        @(posedge cpu_clk);
        first_cpu_edge = $realtime;
        @(posedge cpu_clk);
        second_cpu_edge = $realtime;
        cpu_period_ns = second_cpu_edge - first_cpu_edge;
        $display("[%0t] 实测CPU频率：周期=%.3f ns，频率=%.3f MHz",
                 $time, cpu_period_ns, 1000.0 / cpu_period_ns);
        if (cpu_period_ns <= 0.0)
            $fatal(1, "无法测量PLL的CPU输出时钟");
    end

    // 计时器和UART都以clk_out1为固定50MHz基准，因此只约束这个输出。
    realtime first_ref_edge;
    realtime second_ref_edge;
    realtime ref_period_ns;
    initial begin
        wait (pll_locked === 1'b1);
        @(posedge timer_ref_clk);
        first_ref_edge = $realtime;
        @(posedge timer_ref_clk);
        second_ref_edge = $realtime;
        ref_period_ns = second_ref_edge - first_ref_edge;
        $display("[%0t] 实测计时参考时钟：周期=%.3f ns，频率=%.3f MHz",
                 $time, ref_period_ns, 1000.0 / ref_period_ns);
        if (ref_period_ns < 19.980 || ref_period_ns > 20.020)
            $fatal(1, "计时参考时钟不是50MHz：实测周期%.3f ns", ref_period_ns);
    end

    // 退休级监控：只打印测试里程碑和低频进度，不打印每条指令。
    always @(posedge cpu_clk) begin
        if (pll_locked) begin
            // 直接观察计时器真正接收的CPU域写脉冲。
            if (counter_write && counter_write_data == 32'h8000_0000) begin
                $display("[%0t] 计时器开始：mcycle=%0d，当前=%0d ms",
                         $time, perf_mcycle, counter_ms_value);
            end else if (counter_write &&
                         counter_write_data == 32'hffff_ffff) begin
                $display("[%0t] 计时器停止：mcycle=%0d，实际=%0d ms",
                         $time, perf_mcycle, counter_ms_value);
            end

            if (perf_mcycle >= next_progress_cycle) begin
                $display("[%0t] PROGRESS mcycle=%0d minstret=%0d PC=0x%08h LED=0x%08h SEG=0x%08h",
                         $time, perf_mcycle, perf_minstret,
                         retire_pc, virtual_led, raw_seg_value);
                next_progress_cycle <= next_progress_cycle + 64'd100_000_000;
            end

            if (perf_mcycle >= MAX_CPU_CYCLES)
                $fatal(1, "CPU超过%0d周期仍未到达结束PC，当前PC=0x%08h",
                       MAX_CPU_CYCLES, retire_pc);

            if (retire_count != 0) begin
                if (retire_inst[6:0] == 7'b0110011 &&
                    retire_inst[31:25] == 7'b0000001)
                    rv32m_retire_count <= rv32m_retire_count + 64'd1;

                if (retire_mem_valid && retire_mem_store) begin
                    case (retire_mem_addr)
                        PASS_ADDR: begin
                            if (rv32i_pass_count != retire_mem_wdata)
                                $display("[%0t] RV32I通过数量更新：%0d -> %0d",
                                         $time, rv32i_pass_count,
                                         retire_mem_wdata);
                            rv32i_pass_count <= retire_mem_wdata;
                        end
                        FAIL_ADDR: begin
                            if (retire_mem_wdata != 0)
                                $fatal(1, "程序报告失败：PC=0x%08h，错误值=0x%08h",
                                       retire_pc, retire_mem_wdata);
                        end
                        TEST_COUNT_ADDR: begin
                            if (final_test_count != retire_mem_wdata)
                                $display("[%0t] 决赛测试项通过数量：%0d -> %0d",
                                         $time, final_test_count,
                                         retire_mem_wdata);
                            final_test_count <= retire_mem_wdata;
                        end
                        SEG_ADDR:
                            $display("[%0t] 数码管写入：0x%08h",
                                     $time, retire_mem_wdata);
                        LED_ADDR:
                            $display("[%0t] LED写入：0x%08h",
                                     $time, retire_mem_wdata);
                        COUNTER_ADDR:
                            $display("[%0t] 计时器命令：0x%08h",
                                     $time, retire_mem_wdata);
                    endcase
                end

                if (!cpu_done && retire_pc == END_PC && retire_dnpc == END_PC) begin
                    cpu_done <= 1'b1;
                    $display("[%0t] CPU到达稳定结束循环：PC=0x%08h",
                             $time, retire_pc);
                end
            end
        end
    end
endmodule

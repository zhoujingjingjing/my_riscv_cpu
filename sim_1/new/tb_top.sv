`timescale 1ns / 1ps

// 日常使用的快速分层验收testbench。
// 它不执行耗时的完整性能程序；完整CPU正确性由NEMU DiffTest负责，完整
// XSim终验请把tb_top_full设为Simulation Top。
module tb_top;
    // UART完整往返本身约需24 ms；1000万周期给程序前段留出余量，
    // 同时仍远短于完整irom-v2的491,601,606周期。
    localparam [63:0] FAST_MAX_CPU_CYCLES = 64'd10_000_000;
    localparam [31:0] PASS_ADDR = 32'h8010_0000;
    localparam [31:0] FAIL_ADDR = 32'h8010_0004;
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

    reg [31:0] rv32i_pass_count;
    reg [63:0] rv32m_retire_count;
    reg cpu_fast_check_done;
    reg uart_check_done;

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

    task automatic run_uart_platform_check;
        begin
            // 与官方旧TB相同，先确认无效命令不会触发返回。
            $display("[%0t] UART_CHECK: send invalid command 0x00", $time);
            uart_send_byte(8'h00);
            fork
                begin: UNEXPECTED_TX
                    wait (serial_tx === 1'b0);
                    $fatal(1, "Invalid UART command 0x00 unexpectedly started TX");
                end
                begin: NO_TX_TIMEOUT
                    #100_000;
                    disable UNEXPECTED_TX;
                end
            join

            $display("[%0t] UART_CHECK: set SW[0], SW[31], and KEY[0]", $time);
            uart_send_byte(8'h81);
            #2_000;
            uart_send_byte(8'ha0);
            #2_000;
            uart_send_byte(8'hc1);
            #2_000;

            $display("[%0t] UART_CHECK: read 18-byte platform status", $time);
            fork
                begin
                    uart_send_byte(8'h80);
                end
                begin
                    for (byte_index = 0; byte_index < 18;
                         byte_index = byte_index + 1) begin
                        uart_receive_byte(uart_status[byte_index]);
                        $display("UART_STATUS[%0d] = 0x%02h",
                                 byte_index, uart_status[byte_index]);
                    end
                end
            join

            compare_errors = 0;
            for (byte_index = 0; byte_index < 18;
                 byte_index = byte_index + 1) begin
                if (uart_status[byte_index] !==
                    uut.twin_controller_inst.status_buffer[byte_index]) begin
                    compare_errors = compare_errors + 1;
                    $display("UART mismatch[%0d]: received=0x%02h internal=0x%02h",
                             byte_index, uart_status[byte_index],
                             uut.twin_controller_inst.status_buffer[byte_index]);
                end
            end
            if (compare_errors != 0)
                $fatal(1, "UART status does not match Twin Controller: %0d byte(s)",
                       compare_errors);

            if (uart_status[5][0] !== 1'b1 ||
                uart_status[6][0] !== 1'b1 ||
                uart_status[9][7] !== 1'b1)
                $fatal(1, "UART value for SW[0], KEY[0], or SW[31] is incorrect");

            uart_check_done = 1'b1;
            $display("[%0t] UART_TWIN_CHECK_PASS", $time);
        end
    endtask

    // 只检查程序开头的RV32I结果和RV32M确实进入退休级；完整性能程序由
    // Verilator DiffTest跑完，不能把本快速检查当成完整CPU测试。
    always @(posedge cpu_clk) begin
        if (pll_locked) begin
            if (perf_mcycle >= FAST_MAX_CPU_CYCLES && !cpu_fast_check_done)
                $fatal(1,
                       "Fast CPU check exceeded %0d cycles: PC=0x%08h RV32I=%0d RV32M_RETIRED=%0d",
                       FAST_MAX_CPU_CYCLES, retire_pc,
                       rv32i_pass_count, rv32m_retire_count);

            if (retire_count != 0) begin
                if (retire_inst[6:0] == 7'b0110011 &&
                    retire_inst[31:25] == 7'b0000001)
                    rv32m_retire_count <= rv32m_retire_count + 64'd1;

                if (retire_mem_valid && retire_mem_store) begin
                    if (retire_mem_addr == PASS_ADDR) begin
                        rv32i_pass_count <= retire_mem_wdata;
                        $display("[%0t] RV32I_PASS_COUNT: %0d",
                                 $time, retire_mem_wdata);
                    end
                    if (retire_mem_addr == FAIL_ADDR && retire_mem_wdata != 0)
                        $fatal(1, "CPU program reported failure: PC=0x%08h error=0x%08h",
                               retire_pc, retire_mem_wdata);
                end

                if (!cpu_fast_check_done &&
                    rv32i_pass_count == 32'd37 &&
                    rv32m_retire_count >= 64'd8) begin
                    cpu_fast_check_done <= 1'b1;
                    $display("[%0t] FAST_CPU_CHECK_PASS: RV32I=37 RV32M_RETIRED=%0d",
                             $time, rv32m_retire_count);
                end
            end
        end
    end

    realtime first_cpu_edge;
    realtime second_cpu_edge;
    realtime cpu_period_ns;
    realtime first_ref_edge;
    realtime second_ref_edge;
    realtime ref_period_ns;

    initial begin
        serial_rx = 1'b1;
        rv32i_pass_count = 32'b0;
        rv32m_retire_count = 64'b0;
        cpu_fast_check_done = 1'b0;
        uart_check_done = 1'b0;
        compare_errors = 0;
        for (byte_index = 0; byte_index < 18; byte_index = byte_index + 1)
            uart_status[byte_index] = 8'b0;

        $timeformat(-9, 3, " ns", 16);
        $display("============================================================");
        $display("FAST_VIVADO_PLATFORM_CHECK_START");
        $display("This TB does not replace full NEMU DiffTest or tb_top_full");
        $display("Fast CPU check limit: %0d cycles", FAST_MAX_CPU_CYCLES);
        $display("============================================================");

        wait (pll_locked === 1'b1);

        @(posedge cpu_clk);
        first_cpu_edge = $realtime;
        @(posedge cpu_clk);
        second_cpu_edge = $realtime;
        cpu_period_ns = second_cpu_edge - first_cpu_edge;
        if (cpu_period_ns <= 0.0)
            $fatal(1, "Cannot measure CPU clock");
        $display("CPU_CLOCK: period=%.3f ns frequency=%.3f MHz",
                 cpu_period_ns, 1000.0 / cpu_period_ns);

        @(posedge timer_ref_clk);
        first_ref_edge = $realtime;
        @(posedge timer_ref_clk);
        second_ref_edge = $realtime;
        ref_period_ns = second_ref_edge - first_ref_edge;
        if (ref_period_ns < 19.980 || ref_period_ns > 20.020)
            $fatal(1, "Timer/UART reference clock is not 50 MHz: period=%.3f ns",
                   ref_period_ns);
        $display("REFERENCE_CLOCK: period=%.3f ns frequency=%.3f MHz",
                 ref_period_ns, 1000.0 / ref_period_ns);

        fork
            begin
                run_uart_platform_check();
            end
            begin
                wait (cpu_fast_check_done === 1'b1);
            end
        join

        if (!uart_check_done || !cpu_fast_check_done)
            $fatal(1, "Fast platform check completion flags are inconsistent");

        $display("============================================================");
        $display("RV32I_PASS_COUNT    : %0d / 37", rv32i_pass_count);
        $display("RV32M_RETIRED       : %0d", rv32m_retire_count);
        $display("FINAL_MCYCLE        : %0d", perf_mcycle);
        $display("FINAL_MINSTRET      : %0d", perf_minstret);
        $display("LED                 : 0x%08h", virtual_led);
        $display("SEG                 : 0x%010h", virtual_seg);
        $display("FAST_PLATFORM_TB_PASS");
        $display("============================================================");
        $finish;
    end
endmodule
